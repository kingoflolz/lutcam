"""Generate a vector LFTag3x3 calibration sheet and its physical geometry.

Requires ReportLab. The default is a US Letter sheet with twelve 66 mm tags.
All dimensions include the black outer border. Neighboring tags share one
border, as on the original 36 mm calibration grid.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import random

import reportlab
from reportlab.lib.pagesizes import letter
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen import canvas


ROOT = Path(__file__).resolve().parents[3]
SEED = 0x6C667461675F4C32
DATA_CELLS = ((1, 0), (0, 1), (1, 1), (2, 1), (0, 2), (1, 2), (2, 2))


def rectangles(tag_id):
    """Canonical unrotated black islands in the 24-unit LFTag square."""
    result = [(4, 4, 4, 4), (16, 4, 4, 4)]
    for i, (col, row) in enumerate(DATA_CELLS):
        symbol = (tag_id >> (2 * i)) & 3
        result.append((4 + col * 6 + ((symbol >> 1) & 1),
                       4 + row * 6 + (symbol & 1), 3, 3))
    return result


def rotate(x, y, turns):
    for _ in range(turns):
        x, y = 24 - y, x
    return x, y


def generate(output, rows, columns, width):
    page_w, page_h = letter[0] / mm, letter[1] / mm
    unit, pitch = width / 24, width * 22 / 24
    grid_w, grid_h = width + (columns - 1) * pitch, width + (rows - 1) * pitch
    # Keep all printed elements inside a 1/4-inch printable margin, and reserve
    # space for the title and physical scale check.
    top = 19.0
    if grid_w > page_w - 12.7 or top + grid_h > page_h - 12:
        raise ValueError("grid does not fit US Letter with the title, ruler, and print margins")
    left = (page_w - grid_w) / 2
    rng = random.Random(SEED)
    ids = rng.sample(range(16384), rows * columns)
    turns = [i % 4 for i in range(len(ids))]
    rng.shuffle(turns)
    output.parent.mkdir(parents=True, exist_ok=True)
    pdf = output.with_suffix(".pdf")
    fonts = Path(reportlab.__file__).parent / "fonts"
    pdfmetrics.registerFont(TTFont("GridSans", str(fonts / "Vera.ttf")))
    pdfmetrics.registerFont(TTFont("GridSansBold", str(fonts / "VeraBd.ttf")))
    c = canvas.Canvas(str(pdf), pagesize=letter, pageCompression=1, invariant=1)
    c.setTitle(f"LFTag calibration grid - {len(ids)} tags, {width:g} mm")
    c.setAuthor("Lutcam calibration target")
    c.setSubject(f"LFTag3x3; {width:g} mm outer width; US Letter; print Actual size / 100%")
    c.setViewerPreference("PrintScaling", "None")

    def rect(x, y, w, h, gray):
        c.setFillGray(gray)
        c.rect(x * mm, (page_h - y - h) * mm, w * mm, h * mm, stroke=0, fill=1)

    def label(x, y, text, size=8, bold=False):
        c.setFillGray(0)
        c.setFont("GridSansBold" if bold else "GridSans", size)
        c.drawString(x * mm, (page_h - y) * mm, text)

    label(left, 10.5, "LFTag calibration grid - large tags", 12, True)
    label(left, 14.7,
          f"{len(ids)} unique IDs  |  {columns} x {rows} grid  |  {width:g} mm tag width  |  US Letter", 8)
    rect(left, top, grid_w, grid_h, 0)
    tags = []
    for i, (tag_id, turn) in enumerate(zip(ids, turns)):
        row, col = divmod(i, columns)
        x0, y0 = left + col * pitch, top + row * pitch
        rect(x0 + 2 * unit, y0 + 2 * unit, 20 * unit, 20 * unit, 1)
        shapes = rectangles(tag_id)
        centroids = []
        for x, y, w, h in shapes:
            corners = [rotate(px, py, turn) for px, py in
                       ((x, y), (x + w, y), (x, y + h), (x + w, y + h))]
            xs, ys = zip(*corners)
            rect(x0 + min(xs) * unit, y0 + min(ys) * unit,
                 (max(xs) - min(xs)) * unit, (max(ys) - min(ys)) * unit, 0)
            cx, cy = rotate(x + w / 2, y + h / 2, turn)
            centroids.append([x0 + cx * unit, y0 + cy * unit, 0.0])
        tags.append(dict(row=row, column=col, id=tag_id,
                         rotation_clockwise_deg=turn * 90,
                         center_mm=[x0 + width / 2, y0 + width / 2, 0.0],
                         outer_width_mm=width, centroids_mm=centroids,
                         rectangles_native_24unit_xywh=shapes))

    ruler_y = 272.0
    c.setStrokeGray(0)
    c.setLineWidth(.15 * mm)
    c.line(left * mm, (page_h - ruler_y) * mm,
           (left + 50) * mm, (page_h - ruler_y) * mm)
    for tick in range(0, 51, 10):
        c.line((left + tick) * mm, (page_h - ruler_y - .65) * mm,
               (left + tick) * mm, (page_h - ruler_y + .65) * mm)
    label(left + 7, ruler_y - 1.3, "50 mm - verify after printing", 6.5)
    label(left + 59, ruler_y, "Print at 100% / Actual size. Disable Fit to page.", 8, True)
    c.showPage()
    c.save()

    sources = [Path(__file__).resolve(), ROOT / "software/crates/lftag/src/geometry.rs"]
    metadata = dict(
        schema="lutcam.lftag_calibration_grid.v1", page_size="US Letter",
        page_width_mm=page_w, page_height_mm=page_h, rows=rows, columns=columns,
        grid_pitch_mm=pitch, tag_outer_width_mm=width, native_tag_width_units=24,
        canonical_grid_pitch_mm=width / 4, border_width_mm=2 * unit,
        baseline_square_side_mm=4 * unit, data_square_side_mm=3 * unit,
        coordinate_system="Page top-left origin; +x right, +y down, z=0; rotation is clockwise in this page frame.",
        centroid_order="Two fixed anchors (canonical top-left, top-right), then seven data cells in ascending 2-bit ID-field order.",
        random_seed=SEED, id_range=[0, 16383], ids_unique=True,
        rotation_sampling="Balanced 0,90,180,270 degree rotations, deterministically shuffled.",
        shared_black_border_mm=2 * unit,
        grid_bounds_mm=[left, top, grid_w, grid_h],
        scale_check_mm=50,
        print_instructions="Print on US Letter at Actual size / 100%; verify the 50 mm ruler; mount flat; all tags in a calibration capture must have the same printed size.",
        sources=[dict(path=str(p), sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in sources],
        pdf_sha256=hashlib.sha256(pdf.read_bytes()).hexdigest(), tags=tags)
    output.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(f"Created {pdf}: {columns} x {rows}, {width:g} mm tags, {pitch:g} mm center spacing")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path,
                        default=ROOT / "output/pdf/lftag-calibration-grid-large")
    parser.add_argument("--rows", type=int, default=4)
    parser.add_argument("--columns", type=int, default=3)
    parser.add_argument("--tag-width-mm", type=float, default=66.)
    args = parser.parse_args()
    if args.rows < 1 or args.columns < 1 or args.tag_width_mm <= 0:
        parser.error("rows, columns, and tag width must be positive")
    generate(args.output.resolve(), args.rows, args.columns, args.tag_width_mm)


if __name__ == "__main__":
    main()
