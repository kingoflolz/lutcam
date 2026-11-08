use std::env;
use std::error::Error;
use std::fs::File;
use std::path::{Path, PathBuf};

use image::codecs::gif::{GifEncoder, Repeat};
use image::imageops::FilterType;
use image::{Delay, Frame, ImageReader, Rgb, RgbImage, RgbaImage};
use lrcca::{
    AdaptiveThresholdConfig, TagDetectCentroid, WaveletTagDetectReferenceModel,
    project_wavelet_centroid,
};

struct Config {
    input: PathBuf,
    output: PathBuf,
    gif_output: Option<PathBuf>,
    width: usize,
    height: usize,
    gif_duration_ms: u32,
}

fn main() -> Result<(), Box<dyn Error>> {
    let config = parse_args()?;
    let image = ImageReader::open(&config.input)?.decode()?.to_luma8();
    let image = if image.dimensions() == (config.width as u32, config.height as u32) {
        image
    } else {
        image::imageops::resize(
            &image,
            config.width as u32,
            config.height as u32,
            FilterType::Triangle,
        )
    };
    let tag_config = AdaptiveThresholdConfig::default();
    let model = WaveletTagDetectReferenceModel::new(config.width, config.height, tag_config)?;
    let mut groups = Vec::<Vec<TagDetectCentroid>>::new();
    let mut group = Vec::new();
    for event in model.output_stream(image.as_raw())? {
        if let Some(centroid) = event.centroid {
            group.push(centroid);
        }
        if event.end_inner && !group.is_empty() {
            groups.push(std::mem::take(&mut group));
        }
    }
    if !group.is_empty() {
        return Err("TagDetect stream ended without hier_inner_end".into());
    }

    if let Some(parent) = config.output.parent() {
        std::fs::create_dir_all(parent)?;
    }
    let display_scale = if config.width <= 64 { 16 } else { 1 };
    let base = image::imageops::resize(
        &image,
        (config.width * display_scale) as u32,
        (config.height * display_scale) as u32,
        FilterType::Nearest,
    );
    let mut plot = RgbImage::from_fn(base.width(), base.height(), |x, y| {
        let value = base.get_pixel(x, y).0[0];
        Rgb([value, value, value])
    });
    for group in &groups {
        draw_group(&mut plot, group, config.width, config.height, display_scale);
    }
    plot.save(&config.output)?;

    if let Some(path) = &config.gif_output {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        write_gif(
            path,
            &base,
            &groups,
            config.width,
            config.height,
            display_scale,
            config.gif_duration_ms,
        )?;
    }

    println!("groups: {}", groups.len());
    println!("features: {}", groups.iter().map(Vec::len).sum::<usize>());
    println!("wrote: {}", config.output.display());
    if let Some(path) = config.gif_output {
        println!("wrote: {}", path.display());
    }
    Ok(())
}

fn draw_group(
    image: &mut RgbImage,
    group: &[TagDetectCentroid],
    width: usize,
    height: usize,
    display_scale: usize,
) {
    for &centroid in group {
        let Some((x, y, source_scale)) = project_wavelet_centroid(centroid, width, height) else {
            continue;
        };
        let cx = ((x + 0.5) * display_scale as f64).round() as i32;
        let cy = ((y + 0.5) * display_scale as f64).round() as i32;
        let color = match source_scale {
            1 => Rgb([255, 0, 0]),
            4 => Rgb([0, 255, 0]),
            _ => Rgb([0, 0, 255]),
        };
        draw_marker(image, cx, cy, source_scale, color);
    }
}

fn draw_marker(image: &mut RgbImage, cx: i32, cy: i32, scale: usize, color: Rgb<u8>) {
    let radius = 6i32;
    for delta in -radius - 2..=radius + 2 {
        add_pixel(image, cx + delta, cy, color);
        add_pixel(image, cx, cy + delta, color);
    }
    for y in -radius..=radius {
        for x in -radius..=radius {
            let edge = match scale {
                1 => {
                    let distance = x * x + y * y;
                    ((radius - 1).pow(2)..=(radius + 1).pow(2)).contains(&distance)
                }
                4 => x.abs() == radius || y.abs() == radius,
                _ => y == radius || (y <= radius && (2 * x.abs() + y - radius).abs() <= 1),
            };
            if edge {
                add_pixel(image, cx + x, cy + y, color);
            }
        }
    }
}

fn add_pixel(image: &mut RgbImage, x: i32, y: i32, color: Rgb<u8>) {
    if x < 0 || y < 0 || x >= image.width() as i32 || y >= image.height() as i32 {
        return;
    }
    let pixel = image.get_pixel_mut(x as u32, y as u32);
    for channel in 0..3 {
        pixel.0[channel] = pixel.0[channel].saturating_add(color.0[channel]);
    }
}

#[allow(clippy::too_many_arguments)]
fn write_gif(
    path: &Path,
    base: &image::GrayImage,
    groups: &[Vec<TagDetectCentroid>],
    width: usize,
    height: usize,
    display_scale: usize,
    duration_ms: u32,
) -> Result<(), Box<dyn Error>> {
    let file = File::create(path)?;
    let mut encoder = GifEncoder::new(file);
    encoder.set_repeat(Repeat::Infinite)?;
    let empty = Vec::new();
    for group in if groups.is_empty() {
        std::slice::from_ref(&empty)
    } else {
        groups
    } {
        let mut rgb = RgbImage::from_fn(base.width(), base.height(), |x, y| {
            let value = base.get_pixel(x, y).0[0];
            Rgb([value, value, value])
        });
        draw_group(&mut rgb, group, width, height, display_scale);
        let rgba = RgbaImage::from_fn(rgb.width(), rgb.height(), |x, y| {
            let p = rgb.get_pixel(x, y);
            image::Rgba([p.0[0], p.0[1], p.0[2], 255])
        });
        encoder.encode_frame(Frame::from_parts(
            rgba,
            0,
            0,
            Delay::from_numer_denom_ms(duration_ms, 1),
        ))?;
    }
    Ok(())
}

fn parse_args() -> Result<Config, Box<dyn Error>> {
    let mut args = env::args().skip(1);
    let input = args
        .next()
        .ok_or("usage: tag_detect_visualize INPUT --output PNG [options]")?;
    let mut output = None;
    let mut gif_output = None;
    let mut width = 1280usize;
    let mut height = 800usize;
    let mut gif_duration_ms = 650u32;
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--output" => {
                output = Some(PathBuf::from(
                    args.next().ok_or("--output requires a path")?,
                ))
            }
            "--gif-output" => {
                gif_output = Some(PathBuf::from(
                    args.next().ok_or("--gif-output requires a path")?,
                ))
            }
            "--width" => width = args.next().ok_or("--width requires a value")?.parse()?,
            "--height" => height = args.next().ok_or("--height requires a value")?.parse()?,
            "--gif-duration-ms" => {
                gif_duration_ms = args
                    .next()
                    .ok_or("--gif-duration-ms requires a value")?
                    .parse()?
            }
            "-h" | "--help" => {
                println!(
                    "usage: tag_detect_visualize INPUT --output PNG [--gif-output GIF] [--width N] [--height N] [--gif-duration-ms N]"
                );
                std::process::exit(0);
            }
            other => return Err(format!("unknown argument: {other}").into()),
        }
    }
    Ok(Config {
        input: PathBuf::from(input),
        output: output.ok_or("--output is required")?,
        gif_output,
        width,
        height,
        gif_duration_ms,
    })
}
