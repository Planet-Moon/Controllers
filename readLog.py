from pathlib import Path
import json
import numpy as np
import matplotlib.pyplot as plt
from math import isnan


def main():
    script_path = Path(__file__).parent
    log_file:Path = script_path.joinpath("log.json")
    with open(log_file, "r", encoding="utf-8") as f:
        data:dict = json.load(f)
        x_arr = np.array(data.get("x"))
        e_arr = np.array(data.get("e"))
        u_arr = np.array(data.get("u"))
        v_arr = np.array(data.get("v"))
        t_arr = np.array(data.get("t"))
        fric_arr = np.array(data.get("fric"))
    pass

    fig, ax = plt.subplots(4, 1, constrained_layout=True, sharex=True)
    ax[0].plot(t_arr, x_arr)
    ax[0].set_ylabel("Position")

    ax4 = ax[0].twinx()
    ax4.plot(t_arr, v_arr, color="g")
    ax4.set_ylabel("Velocity", color="g")

    ax[1].plot(t_arr, e_arr)
    ax[1].set_ylabel("Error")

    ax3 = ax[1].twinx()
    ax3.plot(t_arr, u_arr, color="g")
    ax3.set_ylabel("Output", color="g")

    e_arr_log = e_arr
    for i, value in enumerate(e_arr):
        if value and not isnan(value):
            e_arr_log[i] = 20*np.log10(np.abs(value))
        else:
            e_arr_log[i] = -1000

    ax[2].plot(t_arr, e_arr_log, label="Error dB")
    ax[2].legend()

    ax[3].plot(t_arr, fric_arr, label="Friction")
    ax[3].legend()

    plt.show()

if __name__ == "__main__":
    main()
