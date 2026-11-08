//! Frozen behavioral vectors from the retired Python LRCCA reference.

use lrcca::{PreframedHierarchicalTwoPPCReferenceModel, PreframedHierarchyConfig};

type PreframedInputs = (Vec<Vec<u8>>, Vec<Vec<u8>>, Vec<Vec<u8>>, Vec<Vec<usize>>);

fn preframed_case(width: usize, height: usize, holes: &[&[(usize, usize)]]) -> PreframedInputs {
    let mut mask = vec![vec![0; width]; height];
    for row in mask.iter_mut().take(height - 3).skip(3) {
        row[3..width - 3].fill(1);
    }
    for hole in holes {
        for &(x, y) in *hole {
            mask[y][x] = 0;
        }
    }
    let raw = vec![vec![1; width]; height];
    let valid = vec![vec![1; width]; height];
    let source_y = (0..height).map(|y| vec![y; width]).collect();
    (raw, mask, valid, source_y)
}

fn model(width: usize, height: usize) -> PreframedHierarchicalTwoPPCReferenceModel {
    PreframedHierarchicalTwoPPCReferenceModel::new(
        width,
        height,
        PreframedHierarchyConfig {
            centroid_width: Some(width),
            centroid_height: Some(height),
            ..PreframedHierarchyConfig::default()
        },
    )
    .unwrap()
}

#[test]
fn invalid_membership_connects_topology_without_contributing_child_area() {
    let holes: [&[(usize, usize)]; 9] = [
        &[(7, 7)],
        &[(11, 7)],
        &[(15, 7)],
        &[(19, 7)],
        &[(23, 7)],
        &[(7, 11)],
        &[(11, 11)],
        &[(15, 11)],
        &[(19, 11)],
    ];
    let (raw, mask, mut valid, source_y) = preframed_case(32, 24, &holes);
    valid[7][7] = 0;
    let groups = model(32, 24)
        .feature_groups(&raw, &mask, &valid, &source_y)
        .unwrap();
    let features = &groups
        .iter()
        .find(|group| !group.inner_features.is_empty())
        .unwrap()
        .inner_features;

    assert_eq!(
        features
            .iter()
            .map(|feature| (
                feature.area,
                feature.centroid_valid,
                feature.x_q5,
                feature.y_q5
            ))
            .collect::<Vec<_>>(),
        vec![
            (0, false, 0, 0),
            (1, true, 352, 224),
            (1, true, 480, 224),
            (1, true, 608, 224),
            (1, true, 736, 224),
            (1, true, 224, 352),
            (1, true, 352, 352),
            (1, true, 480, 352),
            (1, true, 608, 352),
        ],
    );
}

#[test]
fn ordinary_equal_area_children_keep_python_stable_order() {
    let holes: [&[(usize, usize)]; 9] = [
        &[(7, 7)],
        &[(12, 7), (13, 7)],
        &[(18, 7), (19, 7)],
        &[(24, 7), (25, 7)],
        &[(30, 7), (31, 7)],
        &[(36, 7), (37, 7)],
        &[(7, 14), (8, 14)],
        &[(14, 14), (15, 14)],
        &[(21, 14), (22, 14)],
    ];
    let (raw, mask, valid, source_y) = preframed_case(48, 30, &holes);
    let groups = model(48, 30)
        .feature_groups(&raw, &mask, &valid, &source_y)
        .unwrap();
    let features = &groups
        .iter()
        .find(|group| !group.inner_features.is_empty())
        .unwrap()
        .inner_features;

    assert_eq!(
        features
            .iter()
            .map(|feature| (feature.area, feature.x_q5, feature.y_q5))
            .collect::<Vec<_>>(),
        vec![
            (1, 224, 224),
            (2, 400, 224),
            (2, 592, 224),
            (2, 784, 224),
            (2, 976, 224),
            (2, 1168, 224),
            (2, 240, 448),
            (2, 464, 448),
            (2, 688, 448),
        ],
    );
}

#[test]
fn full_list_equal_key_reuses_canonical_head_like_python() {
    let holes: [&[(usize, usize)]; 10] = [
        &[(7, 7)],
        &[(12, 7), (13, 7)],
        &[(18, 7), (19, 7)],
        &[(24, 7), (25, 7)],
        &[(30, 7), (31, 7)],
        &[(36, 7), (37, 7)],
        &[(42, 7), (43, 7)],
        &[(7, 14), (8, 14)],
        &[(14, 14), (15, 14)],
        &[(21, 14), (22, 14)],
    ];
    let (raw, mask, valid, source_y) = preframed_case(56, 34, &holes);
    let groups = model(56, 34)
        .feature_groups(&raw, &mask, &valid, &source_y)
        .unwrap();
    let group = groups
        .iter()
        .find(|group| !group.inner_features.is_empty())
        .unwrap();

    assert_eq!(
        group
            .inner_features
            .iter()
            .map(|feature| (feature.area, feature.x_q5, feature.y_q5))
            .collect::<Vec<_>>(),
        vec![
            (2, 688, 448),
            (2, 400, 224),
            (2, 592, 224),
            (2, 784, 224),
            (2, 976, 224),
            (2, 1168, 224),
            (2, 1360, 224),
            (2, 240, 448),
            (2, 464, 448),
        ],
    );
}
