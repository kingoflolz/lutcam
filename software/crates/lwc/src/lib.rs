//! Bit-exact software reference for the fixed-rate FPGA page codec.
//!
//! The codec uses a single, fixed profile. There are no alternative
//! wavelet, entropy, scan-order, or container modes: changing any normative
//! choice requires a new codec implementation and identifier.

use std::error::Error;
use std::fs;
use std::path::{Path, PathBuf};

use image::ImageReader;

mod scalar_codec;
mod transform;

pub use scalar_codec::{
    AllocationStats, Codec, CodecError, EncodedFrame, canonical_group_cycle_deltas,
    raw_group_schedule,
};
pub use transform::{FrontendPlane, UnifiedFrontendReference, unified_frontend_reference};

pub const CODEC_NAME: &str = "LWC";
pub const BITS_PER_PIXEL: usize = 1;
pub const FRAME_WIDTH: usize = 1280;
pub const FRAME_HEIGHT: usize = 800;
pub const FRAME_PIXELS: usize = FRAME_WIDTH * FRAME_HEIGHT;

#[derive(Clone, Debug)]
pub struct GrayImage {
    pub width: usize,
    pub height: usize,
    pub pixels: Vec<u8>,
}

impl GrayImage {
    pub fn len(&self) -> usize {
        self.width * self.height
    }

    pub fn is_empty(&self) -> bool {
        self.pixels.is_empty()
    }
}

#[derive(Clone, Debug)]
pub struct NamedGrayImage {
    pub name: String,
    pub image: GrayImage,
}

#[derive(Clone, Copy, Debug)]
pub struct Evaluation {
    pub coded_bits: usize,
    pub sse: u64,
    pub psnr: f64,
    pub mse: f64,
}

#[derive(Clone, Debug)]
pub struct SetSummary {
    pub codec_name: &'static str,
    pub total_pixels: usize,
    pub total_bits: usize,
    pub total_sse: u128,
    pub psnr_sum: f64,
    pub image_count: usize,
}

impl SetSummary {
    pub fn aggregate_mse(&self) -> f64 {
        self.total_sse as f64 / self.total_pixels as f64
    }

    pub fn aggregate_psnr(&self) -> f64 {
        psnr(self.aggregate_mse())
    }

    pub fn mean_psnr(&self) -> f64 {
        self.psnr_sum / self.image_count as f64
    }

    pub fn bits_per_pixel(&self) -> f64 {
        self.total_bits as f64 / self.total_pixels as f64
    }
}

pub fn evaluate_image(original: &GrayImage) -> Result<(Evaluation, Vec<u8>), CodecError> {
    let frame = Codec::encode(&original.pixels, original.width, original.height)?;
    let reconstruction = Codec::decode(&frame)?;
    let sse = sse(&original.pixels, &reconstruction);
    let mse = sse as f64 / original.len() as f64;
    Ok((
        Evaluation {
            coded_bits: frame.payload_bits(),
            sse,
            psnr: psnr(mse),
            mse,
        },
        reconstruction,
    ))
}

pub fn evaluate_set(images: &[NamedGrayImage]) -> Result<SetSummary, CodecError> {
    evaluate_set_impl(images, None)
}

pub fn evaluate_set_with_reconstructions(
    images: &[NamedGrayImage],
    output_dir: &Path,
) -> Result<SetSummary, Box<dyn Error>> {
    fs::create_dir_all(output_dir)?;
    Ok(evaluate_set_impl(images, Some(output_dir))?)
}

fn evaluate_set_impl(
    images: &[NamedGrayImage],
    reconstruction_dir: Option<&Path>,
) -> Result<SetSummary, CodecError> {
    println!("codec {CODEC_NAME}, fixed {BITS_PER_PIXEL}.0 bpp");
    if let Some(output_dir) = reconstruction_dir {
        println!("reconstructed images: {}", output_dir.display());
    }
    println!(
        "{:<10} {:>9} {:>10} {:>9} {:>12} {:>13} {:>12}",
        "image", "size", "bits", "bpp", "PSNR", "MSE", "SSE"
    );

    let mut summary = SetSummary {
        codec_name: CODEC_NAME,
        total_pixels: 0,
        total_bits: 0,
        total_sse: 0,
        psnr_sum: 0.0,
        image_count: 0,
    };

    for named in images {
        let (evaluation, reconstruction) = evaluate_image(&named.image)?;
        if let Some(output_dir) = reconstruction_dir {
            save_gray_pixels(
                &output_dir.join(&named.name),
                named.image.width,
                named.image.height,
                &reconstruction,
            )
            .map_err(|error| CodecError::Io(error.to_string()))?;
        }

        println!(
            "{:<10} {:>4}x{:<4} {:>10} {:>9.4} {:>12.6} {:>13.6} {:>12}",
            named.name,
            named.image.width,
            named.image.height,
            evaluation.coded_bits,
            evaluation.coded_bits as f64 / named.image.len() as f64,
            evaluation.psnr,
            evaluation.mse,
            evaluation.sse,
        );

        summary.total_pixels += named.image.len();
        summary.total_bits += evaluation.coded_bits;
        summary.total_sse += u128::from(evaluation.sse);
        summary.psnr_sum += evaluation.psnr;
        summary.image_count += 1;
    }

    if summary.image_count > 0 {
        println!(
            "{:<10} {:>9} {:>10} {:>9.4} {:>12.6} {:>13.6} {:>12}",
            "aggregate",
            "",
            summary.total_bits,
            summary.bits_per_pixel(),
            summary.aggregate_psnr(),
            summary.aggregate_mse(),
            summary.total_sse,
        );
        println!("mean PSNR: {:.6} dB", summary.mean_psnr());
    }

    Ok(summary)
}

pub fn pngs_in(dir: &Path) -> Result<Vec<PathBuf>, Box<dyn Error>> {
    let mut paths = Vec::new();
    for entry in fs::read_dir(dir)? {
        let path = entry?.path();
        if path
            .extension()
            .and_then(|extension| extension.to_str())
            .is_some_and(|extension| extension.eq_ignore_ascii_case("png"))
        {
            paths.push(path);
        }
    }
    paths.sort();
    Ok(paths)
}

pub fn load_gray_image(path: &Path) -> Result<GrayImage, Box<dyn Error>> {
    let image = ImageReader::open(path)?.decode()?.to_luma8();
    let (width, height) = image.dimensions();
    Ok(GrayImage {
        width: width as usize,
        height: height as usize,
        pixels: image.into_raw(),
    })
}

pub fn load_gray_images(paths: &[PathBuf]) -> Result<Vec<NamedGrayImage>, Box<dyn Error>> {
    paths
        .iter()
        .map(|path| {
            let image = load_gray_image(path)?;
            let name = path
                .file_name()
                .and_then(|name| name.to_str())
                .unwrap_or("<unknown>")
                .to_owned();
            Ok(NamedGrayImage { name, image })
        })
        .collect()
}

fn save_gray_pixels(
    path: &Path,
    width: usize,
    height: usize,
    pixels: &[u8],
) -> Result<(), Box<dyn Error>> {
    let image = image::GrayImage::from_vec(width as u32, height as u32, pixels.to_vec())
        .ok_or("reconstruction pixel count does not match dimensions")?;
    image.save(path)?;
    Ok(())
}

pub fn sse(original: &[u8], reconstruction: &[u8]) -> u64 {
    assert_eq!(original.len(), reconstruction.len());
    original
        .iter()
        .zip(reconstruction)
        .map(|(&left, &right)| {
            let error = i64::from(left) - i64::from(right);
            (error * error) as u64
        })
        .sum()
}

pub fn mse(original: &[u8], reconstruction: &[u8]) -> f64 {
    sse(original, reconstruction) as f64 / original.len() as f64
}

pub fn psnr(mse: f64) -> f64 {
    if mse == 0.0 {
        f64::INFINITY
    } else {
        10.0 * ((255.0 * 255.0) / mse).log10()
    }
}

#[cfg(test)]
mod integration_tests {
    use super::*;
}
