//! Save fixed-rate bitstreams and decoded pixels for an external comparison.
use std::{env, error::Error, fs, path::PathBuf};

use lwc::{Codec, FRAME_HEIGHT, FRAME_PIXELS, FRAME_WIDTH, load_gray_image, pngs_in};

fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<_> = env::args_os().skip(1).collect();
    if args.len() != 2 {
        return Err("usage: encode_dataset SOURCE_PNG_DIRECTORY OUTPUT_DIRECTORY".into());
    }
    let source = PathBuf::from(&args[0]);
    let output = PathBuf::from(&args[1]);
    fs::create_dir_all(&output)?;
    println!("image,bytes,semantic_bits,padding_bits");
    for path in pngs_in(&source)? {
        let input = load_gray_image(&path)?;
        assert_eq!((input.width, input.height), (FRAME_WIDTH, FRAME_HEIGHT));
        let (frame, stats) = Codec::encode_with_stats(&input.pixels, input.width, input.height)?;
        assert_eq!(frame.bytes().len(), FRAME_PIXELS / 8);
        let pixels = Codec::decode(&frame)?;
        let stem = path
            .file_stem()
            .ok_or("missing filename")?
            .to_string_lossy();
        fs::write(output.join(format!("{stem}.lwc")), frame.bytes())?;
        image::save_buffer(
            output.join(format!("{stem}.png")),
            &pixels,
            FRAME_WIDTH as u32,
            FRAME_HEIGHT as u32,
            image::ColorType::L8,
        )?;
        println!(
            "{stem},{},{},{}",
            frame.bytes().len(),
            stats.used_output_bits,
            stats.padding_bits
        );
    }
    Ok(())
}
