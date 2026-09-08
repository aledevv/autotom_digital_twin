Phase 0 — Freeze provenance and define the research contract
Objective

Prevent formulas, units and assumptions from becoming detached from their source.

Required outputs

Create:

docs/research/geometry_inference/
├── 00_problem_definition.md
├── 01_groimp_allometry_audit.md
├── 02_geometry_identifiability.md
├── 03_validation_protocol.md
└── 04_assumption_register.md

Record:

repository commit;
GroIMP input CSV used;
cultivar;
environmental scenarios;
unit conventions;
known validity ranges;
every heuristic and its source.
Acceptance criterion

Every dimension returned by the future system must be traceable to:

direct measurement;
GroIMP allometry;
calibrated structural model;
or explicit fallback constant.

There must be no unlabelled default.

Phase 1 — Define a canonical geometry schema

Create a representation independent of GroIMP and Isaac Sim:

@dataclass
class OrganGeometryEstimate:
    organ_id: str
    organ_class: str
    parent_id: str | None

    rank: int | None
    order: int | None

    centerline_length_m: float | None

    radius_base_m: float | None
    radius_mid_m: float | None
    radius_tip_m: float | None

    ellipse_a_m: float | None
    ellipse_b_m: float | None

    leaf_length_m: float | None
    leaf_width_m: float | None
    leaf_area_m2: float | None

    fruit_axes_m: tuple[float, float, float] | None
    fruit_volume_m3: float | None

    q05: dict[str, float]
    q50: dict[str, float]
    q95: dict[str, float]

    visibility: float
    confidence: float

    source: str
    source_version: str
    extrapolated: bool
    validity_flags: list[str]

Maintain explicit subrecords:

biological_geometry
visual_geometry
physics_geometry
Canonicalization requirement

Canonicalization may normalize orientation and position, but it must not silently destroy metric scale.

Store:

T_world_from_canonical
T_canonical_from_world
metric_scale
camera_or_reconstruction_scale_source
Phase 2 — Stop exporter information loss
Files to modify
src/exporterV2/adapters/groimp_csv/parser.py
src/exporterV2/adapters/groimp_csv/leaf_builder.py
src/exporterV2/adapters/groimp_csv/truss_builder.py
src/exporterV2/core/...
Main-stem change

Replace an average-only branch definition with a segment profile:

"segments": [
    {
        "rank": internode["rank"],
        "length": internode["length"],
        "radius_start": radius_start,
        "radius_end": radius_end,
        "raw_width_m": internode["width_m"],
    }
    for internode in internodes
]

The physics representation may still reduce link count, but the visual and biological representations must preserve every original internode.

Lateral-axis change

Preserve per-internode profiles before any optimizer merges links.

Leaf change

Support explicit fields for:

rachis diameter;
petiolule lengths;
petiolule diameters;
blade length and width;
blade area.

Keep existing ratios only as fallback profiles.

Truss change

Support explicit:

proximal peduncle length and radius;
rachis segment lengths and radii;
individual pedicel lengths and radii;
fruit attachment positions.
GroIMP export additions

Extend the versioned graph export in model/param/auxiliary_tools_and_charts.rgg with missing dimensions such as:

leaf_rachis_diameter
leaf_petiolule_lengths
leaf_petiolule_diameters
truss_rachis_segment_lengths
truss_rachis_radii
pedicel_lengths
pedicel_radii
geometry_schema_version

Do not remove or silently rename existing columns.

Regression test

Using graph_day_40.csv:

each internode’s visual length must match its row;
each internode’s biological radius must match internode_width_m / 2;
no mean reduction is allowed in the biological representation;
physics clamping must not alter stored biological values.
Phase 3 — Extract a versioned GroIMP allometry registry

Create:

src/geometry_inference/
├── __init__.py
├── schema.py
├── allometry.py
├── formula_registry.py
├── units.py
└── provenance.py

configs/
└── groimp_geometry_prior_v1.yaml

Example registry entry:

internode_lower_main:
  organ_class: Internode
  conditions:
    order: 0
    rank_max: 5
  latent_variable:
    name: structural_biomass
    unit: mg
  length:
    form: power_law
    coefficient: 5.072539918
    exponent: 0.236622507
    output_unit: mm
  diameter:
    form: power_law
    coefficient: 4.855196754
    exponent: 0.11083546
    output_unit: mm
  cultivar: Heartbreakers F1 Twiggy Red
  uncertainty:
    residual_sd: null
    parameter_covariance: null
  source:
    file: model/input/dynamic_input/base_plant_dynamic_50.csv
    implementation: model/organs.rgg
  validity:
    status: requires_real_geometry_validation
Formula-parity tests

For a set of biomass values:

calculate dimensions with Python;
compare to GroIMP-exported dimensions;
require floating-point agreement;
test metre/millimetre conversions;
test rank and order dispatch;
test the 20–70 mm petiole validity warning.

This validates fidelity to GroIMP, not biological correctness.

Phase 4 — Implement direct point-cloud measurement

Create:

src/geometry_inference/
├── pointcloud_measurements.py
├── cross_section.py
├── leaf_surface.py
├── fruit_fit.py
├── visibility.py
├── uncertainty.py
└── organ_assignment.py
Cross-section estimator API
estimate_axis_profile(
    points_xyz,
    point_probabilities,
    centerline,
    sample_positions=(0.1, 0.5, 0.9),
    fit_model="ellipse",
) -> AxisGeometryEstimate

Return:

base/middle/tip radii;
ellipse axes;
taper;
residual;
angular coverage;
point density;
bootstrap intervals;
failure reasons.
Leaf estimator API
estimate_leaf_geometry(
    points_xyz,
    confidence_weights,
    skeleton_hint=None,
) -> LeafGeometryEstimate

Return:

surface area;
geodesic or curvature-aware length;
width;
completeness;
boundary uncertainty;
reconstruction method.
Fruit estimator API
estimate_fruit_geometry(
    points_xyz,
    confidence_weights,
    model="ellipsoid",
) -> FruitGeometryEstimate
Synthetic tests

Generate cylinders, tapered tubes, ellipses, curved leaves and ellipsoids with:

varying noise;
missing angular sectors;
outliers;
non-uniform point density.

The estimator must degrade its confidence when information is missing rather than merely returning an unstable value.

Phase 5 — Build the GroIMP simulation bank

Create a reproducible experiment:

src/experiments/geometry_inference/02_groimp_bank/
├── README.md
├── generate_scenarios.py
├── run_groimp_scenarios.py
├── build_bank.py
├── run_generate_scenarios.sh
├── run_build_bank.sh
└── outputs/

For pure-Python steps, use the project environment. For any Isaac-dependent round-trip test, include the established launcher style using:

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
"$HOME/isaacsim/python.sh" "$SCRIPT_DIR/<script>.py"
Scenario dimensions

At minimum:

thermal age or simulation day;
configured densities 35, 50, 65 and 80 plants/m²;
temperature perturbations;
light perturbations;
random seeds;
selected sensitivity-analysis coefficients;
fruit load and ripening stages.

Use a space-filling design rather than a full Cartesian product when dimensionality becomes large.

Bank format

Store one organ per row in Parquet:

scenario_id
plant_id
organ_id
organ_class
parent_id
rank
order
age_dd
plant_age_dd
length_m
diameter_m
area_m2
structural_biomass_mg
subtree_terminal_count
subtree_leaf_count
subtree_leaf_area_m2
path_length_from_root_m
fruit_count
environment_metadata
parameter_vector
Graph features

Compute:

branch order;
rank;
acropetal index;
path distance from root;
parent and child classes;
descendant count;
terminal-organ count;
supported leaf count and area;
supported fruit count and mass;
subtree total path length;
local edge length;
whole-plant height;
total organ count.
Phase 6 — Implement baselines before sophisticated learning

Evaluate these models in order:

Baseline	Description
B0	One constant median per organ class
B1	Rank/order conditional quantile table
B2	Analytic GroIMP allometry
B3	Direct point-cloud geometry only
B4	Interpretable structural regression using graph features
B5	GroIMP simulation-bank nearest-neighbour or ABC posterior
B6	Direct measurement plus GroIMP probabilistic fusion
B7	Learned residual correction
B8	GNN or simulation-based neural posterior, only if justified

The target is not merely to beat B0. The GroIMP component should demonstrate additional value over direct point-cloud measurement.

Phase 7 — Infer latent developmental state

Construct a topology signature such as:

TopologySignature(
    main_stem_internode_count=...,
    lateral_axis_counts_by_rank=...,
    leaves_by_axis_and_rank=...,
    truss_attachment_ranks=...,
    fruits_per_truss=...,
    ripe_fraction=...,
)

Use this to filter simulation-bank candidates.

Then add metric evidence:

total height;
individual skeleton lengths;
measured fruit radii;
visible leaf areas;
measured basal stem diameter;
ripeness;
known planting density or environmental metadata.

Topology alone will usually leave a broad range of candidate states. A few reliable metric anchors can sharply reduce the posterior.

Phase 8 — Calibrate uncertainty

The current GroIMP coefficients do not contain enough stored information to construct academically valid uncertainty intervals.

Required calibration:

Recover original allometry fitting data where available.
Refit log-scale power-law regressions.
Store:
coefficient covariance;
residual variance;
residual covariance between length and diameter;
organ and plant random effects;
validity ranges.
Bootstrap by plant, not by isolated organ.
Evaluate 50%, 80%, 90% and 95% interval coverage.

For two independently fitted allometries,

$$ \log L = \log a+b\log B+\epsilon_L $$ $$ \log D = \log c+d\log B+\epsilon_D $$

the induced length-to-diameter residual is:

$$ \epsilon_D-\frac{d}{b}\epsilon_L $$

Its variance requires:

$$ \sigma_D^2 + \left(\frac{d}{b}\right)^2\sigma_L^2 - 2\frac{d}{b}\operatorname{Cov}(\epsilon_D,\epsilon_L) $$

Without this information, a narrow confidence interval would be fabricated.

Phase 9 — Real-world validation campaign

Use three validation levels.

Level A — Simulator consistency

Input GroIMP-generated skeletons with dimensions hidden.

Objective:

verify that the inverse implementation reproduces the simulator;
debug rank/order and unit handling.

This does not validate biology.

Level B — External tomato datasets

Use TomatoWUR and other open tomato point-cloud resources to test:

internode length;
internode diameter;
architectural graph alignment;
cross-section quality;
cultivar transfer.

The external dataset is not a substitute for target-cultivar validation, but it provides an immediate benchmark.

Level C — Target cultivar and acquisition pipeline

Scan and manually measure the same plants.

For each relevant organ, record:

two orthogonal diameters at base, middle and tip;
internode or axis length;
petiole and rachis dimensions;
pedicel dimensions;
leaf area from a flatbed scan or calibrated image;
fruit axes and mass;
thermal age;
planting density;
relevant light and temperature metadata.

A practical initial engineering pilot could use approximately four developmental stages and six plants per stage. This is not a formal power calculation; use resulting variance and learning curves to determine the final sample size.

The highest measurement priority should be:

truss rachis and proximal peduncle;
pedicels;
petiolules;
lateral branch diameters;
petiole and main-stem diameters;
leaf and fruit dimensions, which are easier to obtain directly.
Data splitting

Split by whole plant, never randomly by organ.

Otherwise, organs from the same plant will appear in both training and testing and produce overly optimistic errors.

Also create at least one holdout by:

developmental stage;
acquisition date;
or environmental condition.
9. Evaluation and decision gates
9.1 Geometric metrics

Report:

MAE in millimetres for radii and lengths;
MAPE where safe;
normalized RMSE;
signed bias;
residuals by rank, order and developmental stage;
per-organ-class interval coverage;
point-to-surface distance;
Chamfer distance;
silhouette overlap from held-out views;
leaf-area and fruit-volume error.

MAPE should not be used for dimensions close to zero.

9.2 Uncertainty metrics

Report:

empirical coverage of q05–q95 intervals;
interval width;
calibration curves;
error as a function of visibility;
error as a function of angular point coverage;
confidence versus actual residual.
9.3 Mechanical metrics

For selected axes:

cantilever tip deflection under known load;
natural frequency;
settling time;
permanent deformation or hysteresis if applicable;
fruit-loaded truss deflection;
detachment force separately from axis stiffness.

Do not allow collision radius clamping to determine biological stiffness implicitly.

9.4 Required ablations

Compare:

topology only;
topology plus rank/order;
topology plus skeleton lengths;
plus developmental stage;
plus environmental metadata;
plus direct point-cloud measurements;
plus GroIMP prior;
fused model.

This will answer your research question empirically:

How much geometric information is actually supplied by topology, and how much comes from direct measurement, developmental context and the FSP model?

9.5 Go/no-go rule

The GroIMP prior is worth integrating if, on held-out real plants, it:

improves incomplete-organ estimates over rank/order lookup;
improves or maintains direct-measurement performance;
reduces catastrophic errors under occlusion;
provides calibrated uncertainty;
transfers acceptably within the intended cultivar and environment.

If it does not, GroIMP should remain a weak plausibility constraint rather than a dimensional estimator.