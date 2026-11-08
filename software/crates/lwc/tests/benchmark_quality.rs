use std::error::Error;
use std::path::Path;

use lwc::{Codec, FRAME_HEIGHT, FRAME_PIXELS, FRAME_WIDTH, load_gray_images, pngs_in, psnr, sse};

#[test]
fn benchmark_set_is_exact_unfiltered_tiling_of_original_set12() -> Result<(), Box<dyn Error>> {
    let originals = load_gray_images(&pngs_in(Path::new("Set12"))?)?;
    let benchmarks = load_gray_images(&pngs_in(Path::new("Set12-1280x800"))?)?;
    assert_eq!(originals.len(), 12);
    assert_eq!(benchmarks.len(), originals.len());

    for (original, benchmark) in originals.iter().zip(&benchmarks) {
        assert_eq!(benchmark.name, original.name);
        assert_eq!(
            (benchmark.image.width, benchmark.image.height),
            (FRAME_WIDTH, FRAME_HEIGHT)
        );
        for y in 0..FRAME_HEIGHT {
            for x in 0..FRAME_WIDTH {
                let expected = original.image.pixels
                    [(y % original.image.height) * original.image.width + x % original.image.width];
                assert_eq!(
                    benchmark.image.pixels[y * FRAME_WIDTH + x],
                    expected,
                    "{} differs from its source tile at ({x},{y})",
                    benchmark.name
                );
            }
        }
    }
    Ok(())
}

#[test]
fn production_clock_bounded_profile_preserves_the_34_db_quality_gate() -> Result<(), Box<dyn Error>>
{
    // The production 5.5 KiB, two-bit-per-clock bucket uses pressure shift 13.
    let images = load_gray_images(&pngs_in(Path::new("Set12-1280x800"))?)?;
    assert_eq!(images.len(), 12);
    let mut total_sse = 0_u128;
    let mut psnr_sum = 0.0;
    for image in &images {
        let frame = Codec::encode(&image.image.pixels, image.image.width, image.image.height)?;
        assert_eq!(frame.payload_bits(), FRAME_PIXELS);
        let reconstruction = Codec::decode(&frame)?;
        let image_sse = sse(&image.image.pixels, &reconstruction);
        total_sse += u128::from(image_sse);
        psnr_sum += psnr(image_sse as f64 / FRAME_PIXELS as f64);
    }
    let aggregate_psnr = psnr(total_sse as f64 / (images.len() * FRAME_PIXELS) as f64);
    let mean_psnr = psnr_sum / images.len() as f64;
    assert_eq!(total_sse, 259_334_641);
    assert!((aggregate_psnr - 34.887_010_313_972_01).abs() < 1e-12);
    assert!((mean_psnr - 35.046_772_862_573_46).abs() < 1e-12);
    assert!(aggregate_psnr > 34.0);
    assert!(mean_psnr > 34.0);
    Ok(())
}
