import easygui
import matplotlib.pyplot as plt
import pandas as pd

telemetry_file_path = easygui.fileopenbox(default="telemetry/*")

with open(telemetry_file_path) as telemetry_csv_file:
    # fig, ax = plt.subplots()
    telemetry = pd.read_csv(telemetry_csv_file, index_col=0, parse_dates=True)
    telemetry['total_error'] = (telemetry['normalized_target_vertical'].abs(
    ) + telemetry['normalized_target_horizontal'].abs()) / 2
    telemetry['normalized_target_horizontal'].plot(
        xlabel="Laikas", ylabel="Santykinė paklaida (nuo -1 iki 1)", ylim=(-1.0, 1.0))
    telemetry['normalized_target_vertical'].plot()
    telemetry['total_error'].plot(linestyle=':')
    print(type(telemetry['normalized_target_vertical']))
    plt.axhline(y=0, color='r', linestyle='--')
    plt.legend(["Horizontali santykinė paklaida",
                "Vertikali santykinė paklaida",
                "Bendra santykinė paklaida",
                "Taikinio kryptis"])

    plt.figure()
    telemetry['distance_to_target'].plot(
        xlabel="Laikas", ylabel="Atstumas (m)")
    plt.axhline(y=1, color='r', linestyle='--')
    plt.legend(["Orlaivio atstumas nuo taikinio", "1 metro riba"])
    plt.show()
