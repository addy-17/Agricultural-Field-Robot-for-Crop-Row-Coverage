# Agricultural Field Configuration and Reference Coordinates

## 1. Field Dimensions

- Field length (X-axis): 26 m
- Field width (Z-axis): 26 m
- Field X boundary: -13 m to +13 m
- Field Z boundary: -13 m to +13 m

## 2. Crop Row Configuration

- Number of crop rows: 4
- Crop row length: 16 m
- Crop row height: 0.8 m
- Crop row width: 0.4 m
- Crop row longitudinal range: X = -8 m to +8 m

### Crop Row Centerlines

| Row | X Center | Z Center |
|---|---:|---:|
| Crop_Row1 | 0 m | -2.7 m |
| Crop_Row2 | 0 m | -0.9 m |
| Crop_Row3 | 0 m | 0.9 m |
| Crop_Row4 | 0 m | 2.7 m |

- Center-to-center row spacing: 1.8 m

## 3. Headland / Turning Areas

The crop rows extend from X = -8 m to X = +8 m, while the field extends from X = -13 m to X = +13 m.

- Left headland: X = -13 m to -8 m
- Left headland width: 5 m
- Right headland: X = +8 m to +13 m
- Right headland width: 5 m

These headland regions provide open space for row-end turning and transition between adjacent rows.

## 4. Static Obstacles

| Obstacle | X | Z |
|---|---:|---:|
| Obstacle_1 | 2.0 m | 0.0 m |

## 5. Robot Start Pose

## 5. Robot Start Pose

The recommended initial robot pose for the agricultural field is:

- X = -7.0 m
- Z = 0.0 m
- Y = to be set according to the Pioneer 3-AT model ground clearance
- Initial heading: along the positive X-axis

The start position is located near the left side of the crop-row region, allowing the robot to enter the field and begin row-coverage navigation.

The final Y-coordinate and robot orientation parameters will be confirmed during integration with Member 2's Pioneer 3-AT robot model.

## 6. Member 1 Handoff

The following environment information is provided to the navigation and coverage-planning modules:

- Field dimensions and boundaries
- Crop-row IDs
- Crop-row centerlines
- Row spacing
- Left and right headland boundaries
- Static obstacle coordinates