import numpy as np
import matplotlib.pyplot as plt
from fk import run_fk
from ik import run_ik

def main():
    print("FK and IK 2 Dof Simulation")
    mode = input("FK or IK? ").strip().lower()

    if mode == "fk":
        run_fk()
    elif mode == "ik":
        run_ik()
    else:
        print("Unknown mode.")


if __name__ == "__main__":
    main()
