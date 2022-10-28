from pathlib import Path
import json
import numpy as np
import matplotlib.pyplot as plt

def main():
    script_path = Path(__file__).parent
    log_file:Path = script_path.joinpath("log.json")
    with open(log_file, "r", encoding="utf-8") as f:
        data:dict = json.load(f)
        x_arr = np.array(data.get("x"))
        e_arr = np.array(data.get("e"))
        u_arr = np.array(data.get("u"))
        t_arr = np.array(data.get("t"))
    pass

    fig, ax = plt.subplots(3, 1, constrained_layout=True, sharex=True)
    ax[0].plot(t_arr, x_arr)
    ax[0].set_ylabel("Position")

    ax[1].plot(t_arr, e_arr)
    ax[1].set_ylabel("Error")

    ax3 = ax[1].twinx()
    ax3.plot(t_arr, u_arr, color="g")
    ax3.set_ylabel("Output", color="g")

    ax[2].plot(t_arr, 20*np.log10(np.abs(e_arr)), label="Error dB")
    ax[2].legend()

    plt.show()

if __name__ == "__main__":
    main()