"""Generate the illustrative M8 holder used by H5 and H7 (CadQuery 2.8).

Only the 20 mm mounting pitch is specified by the design request. The 2.2 mm
mounting bores match the existing PCB. Other dimensions are generic assumptions,
not manufacturing dimensions for a selected commercial holder.
"""
from pathlib import Path
import cadquery as cq

output = Path(__file__).resolve().parent / "Generic_M8_LensHolder_20mm.step"
base = cq.Workplane("XY").box(12, 12, 2.2, centered=(True, True, False))
base = base.edges("|Z").fillet(1)
base = base.union(cq.Workplane("XY").box(6, 20, 2.2, centered=(True, True, False)))
for y in [-10, 10]:
    base = base.union(cq.Workplane("XY").center(0, y).circle(3).extrude(2.2))
body = base.union(cq.Workplane("XY").circle(5).extrude(8))
# Smooth nominal M8 bore; no thread helix or lens is modeled.
body = body.cut(cq.Workplane("XY").circle(4).extrude(9))
# Clearance cavity beneath the holder for the image sensor.
body = body.cut(cq.Workplane("XY").circle(4.5).extrude(1.2))
for y in [-10, 10]:
    body = body.cut(cq.Workplane("XY").center(0, y).circle(1.1).extrude(3))
assembly = cq.Assembly(name="Generic_M8_LensHolder_20mm")
assembly.add(body, name="holder", color=cq.Color(.11, .12, .13))
assembly.export(str(output))
print(output)
