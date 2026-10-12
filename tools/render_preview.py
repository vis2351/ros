"""Draw a quick top view and the geometry seen by the horizontal 2D laser.

This is an SDF inspection aid. The actual map is produced by slam_toolbox in
the running simulator; this image does not replace a Gazebo/RViz screenshot.
"""
from pathlib import Path
import math
import xml.etree.ElementTree as ET

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle


ROOT = Path(__file__).resolve().parents[1]
WORLD = ROOT / "worlds" / "sandbox.world"
OUTPUT = ROOT / "docs" / "landscape_preview.png"


def shapes(model, kind):
    model_pose = [float(value) for value in model.findtext("pose", "0 0 0 0 0 0").split()]
    link = model.find("link")
    if link is None:
        return
    for item in link.findall(kind):
        geometry = item.find("geometry")
        if geometry is None:
            continue
        local_pose = [float(value) for value in item.findtext("pose", "0 0 0 0 0 0").split()]
        pose = [a+b for a,b in zip(model_pose,local_pose)]
        yield pose, geometry, item


def make_patch(pose, geometry, color, alpha=1):
    x, y, z, roll, pitch, yaw = pose
    box = geometry.find("box")
    cylinder = geometry.find("cylinder")
    sphere = geometry.find("sphere")
    if box is not None:
        sx, sy, sz = map(float, box.findtext("size").split())
        patch = Rectangle((x-sx/2, y-sy/2), sx, sy,
                          angle=math.degrees(yaw), rotation_point="center",
                          facecolor=color, edgecolor="none", alpha=alpha)
        return patch, z-sz/2, z+sz/2
    if cylinder is not None:
        radius = float(cylinder.findtext("radius"))
        height = float(cylinder.findtext("length"))
        return Circle((x, y), radius, facecolor=color,
                      edgecolor="none", alpha=alpha), z-height/2, z+height/2
    if sphere is not None:
        radius = float(sphere.findtext("radius"))
        return Circle((x, y), radius, facecolor=color,
                      edgecolor="none", alpha=alpha), z-radius, z+radius
    return None


def main():
    tree = ET.parse(WORLD)
    models = [model for model in tree.findall(".//world/model")
              if model.findtext("static") == "true"]
    fig, axes = plt.subplots(1, 2, figsize=(11, 10),
                             gridspec_kw={"width_ratios": [1, 1]})
    ax_scene, ax_scan = axes

    visuals = []
    for model in models:
        for pose, geometry, item in shapes(model, "visual"):
            rgba = item.findtext("material/diffuse", "0.6 0.6 0.6 1")
            color = tuple(map(float, rgba.split()[:3]))
            made = make_patch(pose, geometry, color)
            if made:
                patch, lower, upper = made
                visuals.append((upper, patch))
    for upper, patch in sorted(visuals, key=lambda entry: entry[0]):
        ax_scene.add_patch(patch)

    # The car scans at about 0.10 m on level road and 0.25 m on the viaduct.
    # Show all surfaces it can encounter over a complete survey lap.
    for model in models:
        for pose, geometry, item in shapes(model, "collision"):
            made = make_patch(pose, geometry, "#202b36")
            if not made:
                continue
            patch, lower, upper = made
            if any(lower <= scan_z <= upper for scan_z in (.10, .25)):
                ax_scan.add_patch(patch)

    for ax in axes:
        ax.set_xlim(-1.72, 1.72)
        ax.set_ylim(-2.78, 2.78)
        ax.set_aspect("equal")
        ax.set_xticks([])
        ax.set_yticks([])
        for spine in ax.spines.values():
            spine.set_visible(False)
    ax_scene.set_title("Gazebo world · top view", fontsize=14)
    ax_scan.set_title("Solid surfaces at LiDAR scan height", fontsize=14)
    ax_scan.set_facecolor("#f3f4f5")
    fig.tight_layout(pad=1.0, w_pad=1.8)
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUTPUT, dpi=180, facecolor="white", bbox_inches="tight")
    print(OUTPUT)


if __name__ == "__main__":
    main()
