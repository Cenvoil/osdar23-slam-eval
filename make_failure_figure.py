import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

OUT = "P1_failure_modes.png"
plt.rcParams.update({
    "font.family": "DejaVu Sans",
    "font.size": 8,
    "figure.dpi": 160,
    "savefig.dpi": 300,
})

rows = [
    ("F1  Scale collapse /\nalong-track drift",
     "Report SE(3) + mileage ratio;\nSim(3) only with a scale gate", "#1f4e79"),
    ("F2  Corridor degeneracy",
     "Inspect along-track error;\ndo not attribute all failures to scale", "#1f4e79"),
    ("F3  Initialization /\nframe accounting",
     "Match by timestamp;\nreport matched frames", "#c55a11"),
    ("F4  Rendering–localization\ndecoupling",
     "Separate dense and\nlocalization tables", "#548235"),
    ("F5  Semantic-source\nsubstitution",
     "Report semantic source\nand mIoU protocol", "#548235"),
]

fig, ax = plt.subplots(figsize=(7.2, 4.6))
ax.set_xlim(0, 1)
ax.set_ylim(0, 1)
ax.axis("off")

top = 0.93
row_h = 0.135
gap = 0.018
left_x, left_w = 0.02, 0.34
right_x, right_w = 0.50, 0.48

ax.text(0.19, 0.985, "Failure mode", ha="center", va="top", fontsize=9, fontweight="bold")
ax.text(0.74, 0.985, "Protocol response", ha="center", va="top", fontsize=9, fontweight="bold")

for i, (label, response, color) in enumerate(rows):
    y = top - (i + 1) * row_h - i * gap
    left = FancyBboxPatch((left_x, y), left_w, row_h,
                          boxstyle="round,pad=0.006,rounding_size=0.012",
                          linewidth=0.8, edgecolor=color, facecolor=color + "18")
    right = FancyBboxPatch((right_x, y), right_w, row_h,
                           boxstyle="round,pad=0.006,rounding_size=0.012",
                           linewidth=0.8, edgecolor="#7f7f7f", facecolor="#f2f2f2")
    ax.add_patch(left); ax.add_patch(right)
    ax.text(left_x + 0.015, y + row_h / 2, label, ha="left", va="center",
            fontsize=8, color="#1a1a1a", linespacing=1.15)
    ax.text(right_x + 0.015, y + row_h / 2, response, ha="left", va="center",
            fontsize=8, color="#1a1a1a", linespacing=1.15)
    arr = FancyArrowPatch((left_x + left_w + 0.012, y + row_h / 2),
                          (right_x - 0.012, y + row_h / 2),
                          arrowstyle="-|>", mutation_scale=9,
                          linewidth=0.8, color="#7f7f7f")
    ax.add_patch(arr)

# Legend for the three families.
for x, c, t in [(0.02, "#1f4e79", "Localization"),
                (0.22, "#c55a11", "Data / protocol"),
                (0.44, "#548235", "Metric / semantics")]:
    ax.add_patch(FancyBboxPatch((x, 0.015), 0.016, 0.016,
                                boxstyle="round,pad=0.002,rounding_size=0.003",
                                linewidth=0, facecolor=c))
    ax.text(x + 0.022, 0.023, t, ha="left", va="center", fontsize=7, color="#333333")

fig.tight_layout(pad=0.2)
fig.savefig(OUT, bbox_inches="tight", facecolor="white")
plt.close(fig)
print("saved", OUT, os.path.getsize(OUT))
