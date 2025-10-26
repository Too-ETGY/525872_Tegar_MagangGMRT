import numpy as np
import matplotlib.pyplot as plt


def inverse_kinematics(x, y, l1, l2):

    r = np.sqrt(x**2 + y**2)

    if r > (l1 + l2) or r < abs(l1 - l2):
        raise ValueError("Target Unreachable")

    cos_q2 = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
    q2_rad_up = np.arccos(cos_q2)
    q2_rad_down = -np.arccos(cos_q2)

    def calc_q1(q2_rad):
        k1 = l1 + l2 * np.cos(q2_rad)
        k2 = l2 * np.sin(q2_rad)
        q1 = np.arctan2(y, x) - np.arctan2(k2, k1)
        return q1

    q1_rad_up = calc_q1(q2_rad_up)
    q1_rad_down = calc_q1(q2_rad_down)

    q1_up, q2_up = np.rad2deg([q1_rad_up, q2_rad_up])
    q1_down, q2_down = np.rad2deg([q1_rad_down, q2_rad_down])

    return (q1_up, q2_up), (q1_down, q2_down)


def run_ik():
    print("\n--- Inverse Kinematics ---")
    l1 = float(input("Femur: "))
    l2 = float(input("Tibia: "))
    x0, y0 = map(float, input("Coxa base position (x, y): ").split(","))
    xt, yt= map(float, input("Target position (x, y): ").split(", "))

    try:
        (q1_up, q2_up), (q1_down, q2_down) = inverse_kinematics(xt - x0, yt - y0, l1, l2)
    except ValueError as e:
        print("Error:", e)
        return

    print(f"\nElbow Up   : q1 = {q1_up:.2f}°, q2 = {q2_up:.2f}°")
    print(f"Elbow Down : q1 = {q1_down:.2f}°, q2 = {q2_down:.2f}°")

    # Draw both configurations
    for (q1, q2, color, label) in [
        (q1_up, q2_up, 'b', 'Elbow Up'),
        (q1_down, q2_down, 'r', 'Elbow Down')
    ]:
        t1 = np.deg2rad(q1)
        t12 = np.deg2rad(q1 + q2)
        jx = x0 + l1 * np.cos(t1)
        jy = y0 + l1 * np.sin(t1)
        x_end = jx + l2 * np.cos(t12)
        y_end = jy + l2 * np.sin(t12)

        plt.plot([x0, jx], [y0, jy], f'{color}-o')
        plt.plot([jx, x_end], [jy, y_end], f'{color}-o', label=label)

    plt.scatter([x0, xt], [y0, yt], c=['k', 'g'])
    plt.text(x0, y0, 'Coxa')
    plt.text(xt, yt, 'Target')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.title("Inverse Kinematics 2-DOF")
    plt.show()
