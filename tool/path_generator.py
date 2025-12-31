import csv
import os

def generate_path_from_nodes(
    node_seq,
    waypoint_dir,
    output_csv="generated_path.csv"
):
    """
    Generate a full path CSV by concatenating waypoint CSV files
    stored in a directory, based on a sequence of nodes.
    """

    routes = [f"{node_seq[i]}_{node_seq[i+1]}.csv"
              for i in range(len(node_seq) - 1)]
    print("Generated route files:", routes)

    full_x, full_y = [], []

    for r in routes:
        csv_path = os.path.join(waypoint_dir, r)

        if not os.path.exists(csv_path):
            print(f"[WARNING] Waypoint file not found: {csv_path}")
            continue

        print(f"Appending {r}")

        with open(csv_path, "r") as f:
            reader = csv.reader(f)
            rows = list(reader)

            if not rows:
                continue

            # --- Detect header ---
            start_idx = 0
            try:
                float(rows[0][0])
                float(rows[0][1])
            except ValueError:
                start_idx = 1  # header exists

            for row in rows[start_idx:]:
                full_x.append(float(row[0]))
                full_y.append(float(row[1]))

    # Write final path CSV
    with open(output_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["X", "Y"])
        for x, y in zip(full_x, full_y):
            writer.writerow([x, y])

    print(f"\n[✔] CSV saved to {output_csv} ({len(full_x)} total points)")


# ===== Example Usage =====
if __name__ == "__main__":
    nodes = [18, 21, 51, 46, 40, 63, 34, 27, 28, 31, 1, 3, 7, 10, 9, 56, 59, 18] # CAV1
    # nodes = [60, 52, 24, 27, 30, 1, 3, 6, 44, 63, 39, 49, 55, 12, 15, 60] # CAV2
    waypoint_dir = "waypoint"  
    output_csv = "p1_12.csv"

    generate_path_from_nodes(nodes, waypoint_dir, output_csv)

