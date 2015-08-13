reset
set key maxrows 3 font ",6"
set grid
plot "TargetPosesInterp.2.data.csv"  u  8:5 with linespoints lt 8 pt 6 ps 1.50 title "Txi2", \
     "TargetPosesInterp.1.data.csv"  u  8:5 with linespoints lt 8 pt 6 ps 2.00 title "Txi1", \
     "CubicSplineStateControlPoses.2.data.csv"  u  8:5 with linespoints lt 7 pt 16 ps 1 title "SC2x", \
     "CubicSplineControlPoses.Pre2.data.csv"  u  8:5 with linespoints lt 4 pt 6 ps 1.50 title "PreC2x", \
     "CubicSplineControlPoses.Pre1.data.csv"  u  8:5 with linespoints lt 3 pt 6 ps 1.00 title "PreC1x", \
     "CubicSplineControlPoses.2.data.csv"  u  8:5 with linespoints lt 4 pt 25 ps 1.50 title "C2x", \
     "CubicSplineControlPoses.1.data.csv"  u  8:5 with linespoints lt 3 pt 25 ps 1.20 title "C1x", \
     "TargetKinFit.Pre2.data.csv" u 20:5 with linespoints lt 4 pt 23 ps 1.50 title "Pre2x", \
     "TargetKinFit.Pre1.data.csv" u 20:5 with linespoints lt 3 pt 23 ps 1.25 title "Pre1x", \
     "TargetKinFit.2.data.csv" u 20:5 with linespoints lt 4 pt 4 ps 1.50 title "2x", \
     "TargetKinFit.1.data.csv" u 20:5 with linespoints lt 3 pt 4 ps 1.20 title "1x"

