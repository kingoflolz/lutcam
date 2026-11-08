use std::env;
use std::error::Error;
use std::fs;
use std::path::PathBuf;

use lwc::{
    CODEC_NAME, Codec, FRAME_HEIGHT, FRAME_WIDTH, evaluate_set, evaluate_set_with_reconstructions,
    load_gray_images, pngs_in,
};

fn main() -> Result<(), Box<dyn Error>> {
    let mut args = env::args().skip(1);
    if args.next().as_deref() == Some("decode") {
        return decode_file(args);
    }
    let config = Config::from_args()?;
    let image_paths = pngs_in(&config.set_dir)?;
    if image_paths.is_empty() {
        return Err(format!("no PNG images found in {}", config.set_dir.display()).into());
    }
    let images = load_gray_images(&image_paths)?;

    if let Some(output_dir) = config.output_dir {
        evaluate_set_with_reconstructions(&images, &output_dir)?;
    } else {
        evaluate_set(&images)?;
    }
    Ok(())
}

fn decode_file(args: impl Iterator<Item = String>) -> Result<(), Box<dyn Error>> {
    let mut paths = Vec::new();
    for arg in args {
        match arg.as_str() {
            "--help" | "-h" => {
                print_usage();
                return Ok(());
            }
            "--compact" => {}
            other if other.starts_with('-') => {
                return Err(format!("unknown decode option: {other}").into());
            }
            _ => paths.push(PathBuf::from(arg)),
        }
    }
    if paths.len() != 2 {
        return Err("decode requires INPUT.lwc OUTPUT.png [--compact]".into());
    }
    let payload = fs::read(&paths[0])?;
    let pixels = Codec::decode_compact(&payload, FRAME_WIDTH, FRAME_HEIGHT)?;
    image::save_buffer(
        &paths[1],
        &pixels,
        FRAME_WIDTH as u32,
        FRAME_HEIGHT as u32,
        image::ColorType::L8,
    )?;
    Ok(())
}

struct Config {
    set_dir: PathBuf,
    output_dir: Option<PathBuf>,
}

impl Config {
    fn from_args() -> Result<Self, Box<dyn Error>> {
        let mut set_dir = PathBuf::from("Set12-1280x800");
        let mut output_dir = Some(PathBuf::from("comp").join(CODEC_NAME));
        let mut args = env::args().skip(1);
        while let Some(arg) = args.next() {
            match arg.as_str() {
                "-h" | "--help" => {
                    print_usage();
                    std::process::exit(0);
                }
                "--set" => {
                    set_dir = PathBuf::from(args.next().ok_or("--set requires a directory")?);
                }
                "--output" => {
                    output_dir = Some(PathBuf::from(
                        args.next().ok_or("--output requires a directory")?,
                    ));
                }
                "--no-output" => output_dir = None,
                other if !other.starts_with('-') => set_dir = PathBuf::from(other),
                other => return Err(format!("unknown argument: {other}").into()),
            }
        }
        Ok(Self {
            set_dir,
            output_dir,
        })
    }
}

fn print_usage() {
    println!("usage: lwc [--set Set12-1280x800] [--output DIR | --no-output]");
    println!("       lwc decode INPUT.lwc OUTPUT.png [--compact]");
    println!("fixed codec: {CODEC_NAME}, exactly 1 bit/pixel before compact transport");
    println!("decode consumes the compact self-terminating image payload");
}
