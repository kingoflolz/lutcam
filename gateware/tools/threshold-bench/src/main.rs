//! Production CDF taps -> adaptive threshold masks -> streaming top-nine CCA -> fixed LFTag.
//! Ground truth is used only after decoding, for one-to-one scoring.

use lftag::fixed::{candidate_from_composite, evaluate_transaction};
use lftag::localize::wavelet_logical_to_source;
use lftag::{Decoder, WeightedCentroid};
use lrcca::hierarchical::{PreframedHierarchicalTwoPPCReferenceModel, PreframedHierarchyConfig};
use lwc::{UnifiedFrontendReference, unified_frontend_reference};
use rayon::prelude::*;
use serde::{Deserialize, Serialize};
use std::{collections::BTreeMap, fs, path::PathBuf, time::Instant};

#[derive(Clone, Deserialize)]
struct Tag {
    id: u16,
    edge_lengths: Vec<f64>,
    centroids: Vec<[f64; 2]>,
}
#[derive(Clone, Deserialize)]
struct Frame {
    index: usize,
    file: String,
    split: String,
    tags: Vec<Tag>,
}
#[derive(Deserialize)]
struct Dataset {
    width: usize,
    height: usize,
    frames: Vec<Frame>,
}

#[derive(Serialize)]
struct Detection {
    id: u16,
    plane: String,
    points: Vec<[f64; 2]>,
    matched_id: Option<u16>,
    duplicate: bool,
}
#[derive(Serialize)]
struct Attempt {
    id: u16,
    plane: String,
    points: Vec<[f64; 2]>,
    log_masses: Vec<u8>,
    accepted: bool,
    rejection: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    counterfactual: Option<serde_json::Value>,
}
#[derive(Serialize)]
struct FrameScore {
    index: usize,
    split: String,
    tp: usize,
    fp: usize,
    duplicates: usize,
    gt: usize,
    bins: [[usize; 2]; 4],
    matched_ids: Vec<u16>,
    detections: Vec<Detection>,
    groups: usize,
    rejects: BTreeMap<String, usize>,
    seconds: f64,
    level_tp: [usize; 3],
    level_fp: [usize; 3],
    agreement: [usize; 3],
    multi_level_tags: usize,
    level_bins: [[usize; 4]; 3],
    attempts: Vec<Attempt>,
}
#[derive(Default, Serialize)]
struct Totals {
    images: usize,
    gt: usize,
    tp: usize,
    fp: usize,
    duplicates: usize,
    bins: [[usize; 2]; 4],
    level_tp: [usize; 3],
    level_fp: [usize; 3],
    agreement: [usize; 3],
    multi_level_tags: usize,
    level_bins: [[usize; 4]; 3],
}

fn geometry_error(a: &[[f64; 2]], b: &[[f64; 2]]) -> f64 {
    let directed = |x: &[[f64; 2]], y: &[[f64; 2]]| {
        x.iter()
            .map(|p| {
                y.iter()
                    .map(|q| (p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2))
                    .fold(f64::INFINITY, f64::min)
            })
            .sum::<f64>()
            / x.len() as f64
    };
    directed(a, b).max(directed(b, a)).sqrt()
}

fn score(
    frame: &Frame,
    front: &UnifiedFrontendReference,
    counterfactuals: bool,
) -> Result<FrameScore, String> {
    let start = Instant::now();
    let composite = lrcca::build_wavelet_composite(front, &Default::default())?;
    let hierarchy = PreframedHierarchicalTwoPPCReferenceModel::new(
        composite.width,
        composite.height,
        PreframedHierarchyConfig {
            centroid_width: Some(composite.width),
            centroid_height: Some(front.stage0.height),
            use_coarse_child_key: false,
            use_full_log_mass_child_key: true,
            midpoint_reflection_reciprocal: true,
        },
    )?;
    let groups = hierarchy.feature_groups(
        &composite.raw,
        &composite.mask,
        &composite.feature_valid,
        &composite.source_y,
    )?;
    let group_count = groups.len();
    let decoder = Decoder::default();
    let mut detections = Vec::new();
    let mut rejects = BTreeMap::new();
    let mut attempts = Vec::new();
    let mut matched = vec![false; frame.tags.len()];
    let mut level_masks = vec![0_u8; frame.tags.len()];
    let mut level_fp = [0; 3];
    let mut fp = 0;
    let mut duplicates = 0;
    for group in groups {
        if group.inner_features.is_empty() {
            continue;
        }
        if group.inner_features.len() != 9 {
            return Err("production shortlist is not nine entries".into());
        }
        let centroids = std::array::from_fn(|i| {
            let f = group.inner_features[i];
            WeightedCentroid {
                x_q4: (f.x_q5 >> 1) as i32,
                y_q4: (f.y_q5 >> 1) as i32,
                log_mass_q5_3: f.log_m_q5_3,
                valid: f.centroid_valid,
            }
        });
        let candidate = match candidate_from_composite(centroids, 0) {
            Ok(c) => c,
            Err(e) => {
                *rejects.entry(format!("coordinate_{e:?}")).or_default() += 1;
                continue;
            }
        };
        let result = evaluate_transaction(&decoder, &candidate);
        let points = candidate
            .centroids
            .iter()
            .map(|p| {
                let q = wavelet_logical_to_source(p.point(), candidate.plane);
                [q.x, q.y]
            })
            .collect::<Vec<_>>();
        // Optional ground-truth diagnostics run only after the unmodified
        // production decoder. They never affect matching, detection, or ranking.
        let counterfactual = if counterfactuals {
            frame.tags.iter().find(|t|geometry_error(&points,&t.centroids)<=(t.edge_lengths.iter().copied().fold(0.,f64::max)*0.04).max(2.0)).and_then(|tag|{
                let assignment=points.iter().map(|p|tag.centroids.iter().enumerate().min_by(|(_,a),(_,b)|{
                    let distance=|q:&[f64;2]|(p[0]-q[0]).powi(2)+(p[1]-q[1]).powi(2);
                    distance(a).total_cmp(&distance(b))
                }).unwrap().0).collect::<Vec<_>>();
                let mut unique=assignment.clone();unique.sort_unstable();unique.dedup();
                if unique.len()!=9{return None;}
                let evaluate=|ideal:bool,anchors:bool|{
                    let mut modified=candidate.clone();
                    for (i,&gt) in assignment.iter().enumerate(){
                        if ideal {
                            let scale=f64::from(candidate.plane.scale());
                            let phase=if candidate.plane==lftag::Plane::Ll4 {4.}else{0.};
                            modified.centroids[i].x_q4=(tag.centroids[gt][0]/scale*16.).round() as i32;
                            modified.centroids[i].y_q4=((tag.centroids[gt][1]-phase)/scale*16.).round() as i32;
                        }
                        if anchors {modified.centroids[i].log_mass_q5_3=if gt<2 {91+gt as u8}else{80};}
                    }
                    let r=evaluate_transaction(&decoder,&modified);
                    serde_json::json!({"correct":r.accepted&&r.id==tag.id,"id":r.id,"accepted":r.accepted,"rejection":format!("{:?}",r.reject_reason)})
                };
                Some(serde_json::json!({"gt_id":tag.id,"ideal_centroids":evaluate(true,false),"forced_anchor_masses":evaluate(false,true),"ideal_centroids_and_anchors":evaluate(true,true)}))
            })
        } else {
            None
        };
        attempts.push(Attempt {
            id: result.id,
            plane: format!("{:?}", result.plane),
            points: points.clone(),
            log_masses: candidate
                .centroids
                .iter()
                .map(|p| p.log_mass_q5_3)
                .collect(),
            accepted: result.accepted,
            rejection: format!("{:?}", result.reject_reason),
            counterfactual,
        });
        if !result.accepted {
            *rejects
                .entry(format!("{:?}", result.reject_reason))
                .or_default() += 1;
            continue;
        }
        let level = match result.plane {
            lftag::Plane::Raw => 0,
            lftag::Plane::Ll2 => 1,
            lftag::Plane::Ll4 => 2,
        };
        let gt = frame.tags.iter().enumerate().find(|(_, t)| {
            t.id == result.id
                && geometry_error(&points, &t.centroids)
                    <= (t.edge_lengths.iter().copied().fold(0., f64::max) * 0.04).max(2.0)
        });
        let (matched_id, duplicate) = if let Some((index, tag)) = gt {
            let duplicate = matched[index];
            matched[index] = true;
            level_masks[index] |= 1 << level;
            if duplicate {
                duplicates += 1;
            }
            (Some(tag.id), duplicate)
        } else {
            fp += 1;
            level_fp[level] += 1;
            (None, false)
        };
        detections.push(Detection {
            id: result.id,
            plane: format!("{:?}", result.plane),
            points,
            matched_id,
            duplicate,
        });
    }
    let mut bins = [[0; 2]; 4];
    let mut level_tp = [0; 3];
    let mut agreement = [0; 3];
    let mut level_bins = [[0; 4]; 3];
    for ((tag, &found), &mask) in frame.tags.iter().zip(&matched).zip(&level_masks) {
        let size = tag.edge_lengths.iter().copied().fold(0., f64::max);
        let bin = if size < 32. {
            0
        } else if size < 64. {
            1
        } else if size < 128. {
            2
        } else {
            3
        };
        bins[bin][0] += usize::from(found);
        bins[bin][1] += 1;
        if mask != 0 {
            agreement[mask.count_ones() as usize - 1] += 1;
        }
        for level in 0..3 {
            if mask & (1 << level) != 0 {
                level_tp[level] += 1;
                level_bins[level][bin] += 1;
            }
        }
    }
    let matched_ids = frame
        .tags
        .iter()
        .zip(&matched)
        .filter_map(|(t, &m)| m.then_some(t.id))
        .collect::<Vec<_>>();
    Ok(FrameScore {
        index: frame.index,
        split: frame.split.clone(),
        tp: matched_ids.len(),
        fp,
        duplicates,
        gt: frame.tags.len(),
        bins,
        matched_ids,
        detections,
        groups: group_count,
        rejects,
        seconds: start.elapsed().as_secs_f64(),
        level_tp,
        level_fp,
        agreement,
        multi_level_tags: agreement[1] + agreement[2],
        level_bins,
        attempts,
    })
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = std::env::args().skip(1).collect::<Vec<_>>();
    let mut values = BTreeMap::new();
    let mut counterfactuals = false;
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--counterfactuals" => {
                counterfactuals = true;
                i += 1;
            }
            "--dataset" | "--output" | "--limit" | "--split" | "--frame-indices" => {
                let value = args.get(i + 1).ok_or("missing argument value")?;
                values.insert(args[i].clone(), value.clone());
                i += 2;
            }
            "--help" | "-h" => {
                println!(
                    "threshold-bench --dataset DIR --output DIR [--split NAME] [--limit N] [--frame-indices 0,1] [--counterfactuals]"
                );
                return Ok(());
            }
            value => return Err(format!("unknown argument: {value}").into()),
        }
    }
    let root = PathBuf::from(values.get("--dataset").ok_or("--dataset required")?);
    let output = PathBuf::from(values.get("--output").ok_or("--output required")?);
    fs::create_dir_all(&output)?;
    let dataset: Dataset = serde_json::from_str(&fs::read_to_string(root.join("manifest.json"))?)?;
    let limit = values
        .get("--limit")
        .map(|s| s.parse::<usize>())
        .transpose()?
        .unwrap_or(usize::MAX);
    let indices = values
        .get("--frame-indices")
        .map(|s| {
            s.split(',')
                .map(str::parse::<usize>)
                .collect::<Result<Vec<_>, _>>()
        })
        .transpose()?;
    let frames = dataset
        .frames
        .iter()
        .filter(|f| values.get("--split").is_none_or(|s| s == &f.split))
        .filter(|f| indices.as_ref().is_none_or(|ids| ids.contains(&f.index)))
        .take(limit)
        .collect::<Vec<_>>();
    let all = frames
        .par_iter()
        .map(|frame| -> Result<FrameScore, String> {
            let pixels = image::open(root.join(&frame.file))
                .map_err(|e| e.to_string())?
                .into_luma8();
            if pixels.dimensions() != (dataset.width as u32, dataset.height as u32) {
                return Err("image dimensions differ".into());
            }
            let front = unified_frontend_reference(pixels.as_raw(), dataset.width, dataset.height)?;
            if let Ok(directory) = std::env::var("LUTCAM_DUMP_MASKS") {
                let directory = PathBuf::from(directory);
                fs::create_dir_all(&directory).map_err(|e| e.to_string())?;
                let planes = lrcca::adaptive_threshold_planes(&front)?;
                for (name, values) in [
                    (
                        "base",
                        planes.raw_base_mask.iter().map(|&p| p * 255).collect(),
                    ),
                    (
                        "reference",
                        planes.raw_thresholds.iter().map(|&p| p as u8).collect(),
                    ),
                ] {
                    image::GrayImage::from_raw(dataset.width as u32, dataset.height as u32, values)
                        .unwrap()
                        .save(directory.join(format!("adaptive_{:03}_{name}.png", frame.index)))
                        .map_err(|e| e.to_string())?;
                }
                for (level, plane) in [&front.stage0, &front.ll2, &front.ll4].iter().enumerate() {
                    image::ImageBuffer::<image::Luma<u16>, Vec<u16>>::from_vec(
                        plane.width as u32,
                        plane.height as u32,
                        plane.samples.iter().map(|&p| (p + 32768) as u16).collect(),
                    )
                    .unwrap()
                    .save(directory.join(format!("frame_{:03}_tap{level}_i16.png", frame.index)))
                    .map_err(|e| e.to_string())?;
                    image::GrayImage::from_raw(
                        plane.width as u32,
                        plane.height as u32,
                        planes.masks[level].iter().map(|&p| p * 255).collect(),
                    )
                    .unwrap()
                    .save(directory.join(format!("adaptive_{:03}_{level}.png", frame.index)))
                    .map_err(|e| e.to_string())?;
                }
            }
            let result = score(frame, &front, counterfactuals)?;
            eprintln!(
                "frame {:03}: {}/{} fp={}",
                frame.index, result.tp, result.gt, result.fp
            );
            Ok(result)
        })
        .collect::<Result<Vec<_>, _>>()?;
    let mut totals: BTreeMap<String, Totals> = BTreeMap::new();
    for r in &all {
        for key in ["all", r.split.as_str()] {
            let t = totals.entry(key.to_string()).or_default();
            t.images += 1;
            t.gt += r.gt;
            t.tp += r.tp;
            t.fp += r.fp;
            t.duplicates += r.duplicates;
            for bin in 0..4 {
                for v in 0..2 {
                    t.bins[bin][v] += r.bins[bin][v];
                }
            }
            t.multi_level_tags += r.multi_level_tags;
            for level in 0..3 {
                t.level_tp[level] += r.level_tp[level];
                t.level_fp[level] += r.level_fp[level];
                t.agreement[level] += r.agreement[level];
                for bin in 0..4 {
                    t.level_bins[level][bin] += r.level_bins[level][bin];
                }
            }
        }
    }

    let result = serde_json::json!({"config":{"name":"adaptive"}, "totals":totals,
        "bin_edges":[20,32,64,128,250],
        "matching":"correct ID plus symmetric nearest-centroid RMS <= max(2px, 4% max edge); one-to-one across scales; duplicates reported separately",
        "decoder":"production fixed point",
        "hierarchy":"production top-nine streaming hierarchy",
        "levels":["RAW","LL2","LL4"],
        "agreement_bins":["exactly_one_level","exactly_two_levels","all_three_levels"],
        "frames":all});
    fs::write(
        output.join("adaptive.json"),
        serde_json::to_string_pretty(&result)? + "\n",
    )?;
    println!("adaptive {}", serde_json::to_string(&result["totals"])?);
    Ok(())
}
