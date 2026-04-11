# ParaBridge: Parametric Steel Girder Bridge CAD Model

This project generates a fully parametric 3D CAD model of a short-span steel girder bridge using the `pythonOCC` library. It was developed to fulfill the design requirements for the FOSSEE Osdag bridge module.

## Project Structure
- `bridge_model.py`: The core executable script. It contains all configurable geometric parameters at the top of the file, implements wrapper factory functions for TopoDS solid generation, and handles the full component assembly and visualization.
- `draw_i_section.py`: Base FOSSEE geometry primitive for steel I-beams.
- `draw_rectangular_prism.py`: Base FOSSEE geometry primitive for concrete slabs.
- `tests/test_model.py`: Validation script to ensure reliable logic execution and expected parameter behavior.

## Key Features
- **Parameterized Geometry**: Modify variables like `span_length_L`, `n_girders`, or `deck_thickness` directly at the top of the script.
- **FOSSEE Compliant Detailing**: Includes hammerhead pier caps, grouped pilings with stirrup rings, diagonal cross-frames, and layered RC deck reinforcement.
- **Material Rendering**: Concrete parts render semi-transparently (alpha=0.35), revealing the full opaque steel reinforcement grid inside the deck slab.
- **Automated Export**: Saves a `.step` file automatically on every successful assembly.

## Installation

It is highly recommended to run this in a `conda` environment, as compiling pythonOCC from scratch can be difficult.

```bash
conda create -n parabridge python=3.10
conda activate parabridge
conda install -c conda-forge pythonocc-core
pip install -r requirements.txt
```

## Usage

To assemble the bridge, export the STEP file, and launch the interactive 3D viewer, run:

```bash
python bridge_model.py
```

**(Optional) CLI overrides:**
You can temporarily test different span geometries using CLI arguments:
```bash
python bridge_model.py --span 18000
```

## Running Tests

To verify that the bridge generates a mathematically valid TopoDS topological compound containing the correct component groupings, use pytest:

```bash
pytest tests/
```
