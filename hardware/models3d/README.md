# Lutcam component 3D models

Dimensions are millimetres. Models use the footprint origin, with the PCB mounting surface at Z = 0. Model transforms are baked into these files; attach them with scale 1 and zero offset/rotation.

## Sources and fidelity

| Model | References | Source / fidelity |
|---|---|---|
| R-RJ45S08P-B000 | J1 | Supplier CAD for LCSC C386763; aligned to the eight contact pins and shield stakes. |
| GT-USB-7055BB | J2 | Supplier CAD for LCSC C7450621; 6.5 mm above PCB, seating plane aligned to SMD contacts. |
| YXC_OSC_1612 | Y1 | Supplier CAD for LCSC C41371106; generic 1.6 × 1.2 mm package representation. |
| G2419S | TR1 | Supplier CAD for LCSC C3003220; rotated to the footprint’s long Y axis. |
| ABS210 | D1, D3 | Supplier CAD for LCSC C42406052; aligned by lead positions and polarity. |
| D_PowerDI-5 | D4 | Supplier CAD for LCSC C154881 (PDS3100-13); existing KiCad package origin. |
| TI60F225C4 | IC1 | Simplified package generated from Efinix Titanium Packaging User Guide v2.0, page 18: 10 × 10 mm, 15 × 15 balls at 0.65 mm pitch; nominal modeled height 0.93 mm (datasheet maximum 1.0 mm). |
| OV9281_CSP64 | U8, U11 | Simplified package from the OV9281 datasheet, sections 9–10: 5.237 × 4.463 × 0.715 mm; 64 balls at 0.64 × 0.50 mm pitch. Package center is (-0.2475, +0.0360) in model XY, preserving the optical-center footprint origin. Glass and active area are illustrative. |
| PSPMAA0605H | L4 | Simplified molded inductor, nominal 7.1 × 6.6 mm body and maximum 5.0 mm height. Terminations and corner radii are illustrative. |
| Si3404_QFN20 | U6 | Simplified package from Skyworks Si3404 datasheet, table 6.1: 4 × 4 × 0.85 mm, 0.5 mm lead pitch, 2.6 × 2.6 mm exposed pad. |
| Generic_M8_LensHolder_20mm | H5, H7 | Generic M8 holder with 20 mm mounting centers; preserves the existing 2.2 mm PCB holes. Assumed 12 × 26 × 8 mm envelope, 10 mm barrel diameter, 2.2 mm base thickness, and 9 mm diameter × 1.2 mm deep underside sensor cavity. |

The generated models are visual/mechanical package representations, not manufacturer-certified enclosure tolerance models. For enclosure clearances, use the component datasheet maximum dimensions and tolerances. Downloaded models remain third-party supplier CAD with their original ownership.

Public supplier metadata: `https://lceda.cn/api/products/<LCSC-ID>/components?version=6.4.19.5`. Model UUIDs and download URLs are recorded in `sources.json`.

The supplier `.wrl` files preserve their colors. Matching `.step` files contain the same aligned geometry for CAD export; their surfaces are gray. STEP board export should enable model substitution (`--subst-models`) to use STEP counterparts of VRML files.

Most standard passives and ICs retain their existing `${KICAD10_3DMODEL_DIR}` references and use the installed KiCad model library. The four remaining bare mounting-hole footprints and two net-tie footprints have no separate physical component body and need no model.

The root-level `TI60F225C4.stp` is the same model as `models3d/TI60F225C4.step`; its location satisfies the library footprint’s relative model reference. The placed IC1 uses the explicit project-relative model path.

## Validation

The M8 holders are H5 over U8 and H7 over U11. Both holders are on B.Cu, centered at (110, 120) and (240, 120) mm. Each preserves all 18 original mounting pads, their drill sizes, UUIDs, and global positions. The shared camera schematic represents one holder per camera instance. Bare mounting holes H1–H4 remain separate. There are now 215 physical components with resolved model files and six footprints without a separate body.

The holder is an illustrative model, not a selected supplier part. Its 8 mm bore is smooth; lens threads, lens, and screws are omitted. Body dimensions, height, and sensor cavity are assumptions. The model can be regenerated with `generate_m8_holder.py` using CadQuery. The footprint is `OV9281:MountingHole_M8_LensHolder_P20mm_2xM2_Pad_Via`; the holders retain mechanical BOM exclusion and are excluded from pick-and-place output.

