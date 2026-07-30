# Bicycle-model figure package

These figures mirror the reporting sequence used in `AFS_vs_TV/results/CarSim_DLC70_mu090`.

Each scenario contains figures 01-10 and 12 in both PNG and vector PDF format. Figures 11_1 and 11_2 are omitted because they verify the CarSim steering interface and do not have a bicycle-model equivalent.

Controller colors are consistent across all plots: Baseline is grey, AFS is red, TV is green, and the reference is blue.

Baseline and AFS traces are taken from the committed validated comparison results. TV traces are freshly simulated using the current `TV.slx` implementation. Yaw-rate comparisons use the same unfiltered, friction-limited path target for all controllers.

`Bicycle_Model_metrics_summary.csv` contains the numerical comparison and `Bicycle_Model_plot_data.mat` contains the plotted signals.
