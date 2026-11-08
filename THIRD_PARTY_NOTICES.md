# Third-party material

The Apache-2.0 license in [LICENSE](LICENSE) applies to Lutcam's original
work, including its RTL, software, firmware, and hardware designs. It does
not replace third-party licenses, copyright notices, or restrictions.
Existing third-party notices remain applicable, including when material is
embedded in a larger file, modified, or converted to another format.

## Vendored code and incorporated source

| Material | License and attribution |
| --- | --- |
| `gateware/vendor/luna/` | BSD-3-Clause; see its [LICENSE](gateware/vendor/luna/LICENSE), per-file notices, and [provenance](gateware/vendor/luna/VENDORED.md). |
| `gateware/vendor/litex_luna/` | BSD-2-Clause; see its [LICENSE](gateware/vendor/litex_luna/LICENSE) and [provenance](gateware/vendor/litex_luna/VENDORED.md). |
| `gateware/vendor/liteeth/` | BSD-2-Clause; see its [LICENSE](gateware/vendor/liteeth/LICENSE), per-file notices, and [provenance](gateware/vendor/liteeth/VENDORED.md). |
| `gateware/vendor/picorv32/` | ISC; see [COPYING](gateware/vendor/picorv32/COPYING) and [provenance](gateware/vendor/picorv32/README.lutcam). |
| LiteX-derived UART logic in `gateware/fpga/litex/lutcam/record_bus.py` | BSD-2-Clause; the upstream copyright notices and license text are retained [in the source](gateware/fpga/litex/lutcam/record_bus.py). |

The vendored copies, including their local changes, retain the licenses
provided in those directories.

## Hardware libraries, models, and documentation

- Standard KiCad library elements retain their
  [CC-BY-SA-4.0 license with the KiCad libraries exception](https://www.kicad.org/libraries/license/).
  The exception permits their use in designs under other licenses; it does
  not relicense the library elements themselves.
- `hardware/TI60F225/TI60F225C4.kicad_sym` identifies SamacSys as its generator.
  That symbol, the associated supplied footprint, and copies of third-party
  elements embedded in design files are not relicensed here. Any applicable
  supplier permissions and [SamacSys terms](https://componentsearchengine.com/TermsOfUse.html)
  remain relevant; no separate redistribution grant is recorded here.
- Downloaded supplier models in `hardware/models3d/`, including converted
  STEP and VRML versions, retain their original ownership and terms.
  [Model provenance](hardware/models3d/README.md) and
  [source URLs](hardware/models3d/sources.json) identify these files.
  A public download URL is not a grant of Apache-2.0 rights.
- Manufacturer documents in `hardware/datasheets/` and the supplier pinout
  spreadsheet `hardware/TI60F225/ti60_pinout-v1.1.xlsx` are third-party
  reference material. They are excluded from Lutcam's Apache-2.0 grant.
  In particular, the bundled OmniVision OV9281 specification prohibits
  redistribution, and the Realtek RTL8211F datasheet is marked confidential
  and requires permission for reproduction. Other manufacturer documents
  also retain their own terms; see the [datasheet inventory](hardware/datasheets/README.md).

Lutcam's licensing does not establish permission to redistribute supplier
files. Those permissions must be established separately before distributing
the affected material.

## Dependencies and external tools

Rust and Python dependencies retain their respective licenses. The versions
are recorded in [Cargo.lock](Cargo.lock) and [gateware/uv.lock](gateware/uv.lock).
Distributions that include dependencies must carry their applicable notices
and satisfy any source-availability or other license requirements. This file
is a scope and provenance guide, not a complete notice bundle for a compiled
application or a packaged development environment.

Efinity and its vendor models are separately licensed by Efinix. Lutcam's
Apache-2.0 license does not grant rights to redistribute those tools or models.

Third-party images, datasets, and reference implementations used by tests or
benchmarks retain their own terms. In particular, the optional
[codec comparison script](software/crates/lwc/tools/reproduce_comparison.sh)
downloads [DIV2K](https://data.vision.ee.ethz.ch/cvl/DIV2K/), offered for academic
research only, and ISO JPEG XS reference software with a limited
evaluation/conformance license. Neither is relicensed by this repository.
