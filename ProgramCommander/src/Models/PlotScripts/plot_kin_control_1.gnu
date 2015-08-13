reset
set key maxrows 3 font ",6"
plot "TargetPoses.4.data.csv"  u  8:5 with linespoints lt 8 pt 6 ps 1.5 title "Tx", \
     "TargetPosesInterp.4.data.csv"  u  8:5 with linespoints lt 8 pt 9 title "Txi", \
     "CubicSplineStateControlPoses.4.data.csv"  u  8:5 with linespoints lt 7 pt 16 ps 1 title "SC4x", \
     "CubicSplineControlPoses.Pre4.data.csv"  u  8:5 with linespoints lt 9 pt 6 ps 2.00 title "PreC4x", \
     "CubicSplineControlPoses.Pre3.data.csv"  u  8:5 with linespoints lt 7 pt 6 ps 1.80 title "PreC3x", \
     "CubicSplineControlPoses.Pre2.data.csv"  u  8:5 with linespoints lt 4 pt 6 ps 1.60 title "PreC2x", \
     "CubicSplineControlPoses.Pre1.data.csv"  u  8:5 with linespoints lt 3 pt 6 ps 1.40 title "PreC1x", \
     "CubicSplineControlPoses.4.data.csv"  u  8:5 with linespoints lt 9 pt 25 ps 2.00 title "C4x", \
     "CubicSplineControlPoses.3.data.csv"  u  8:5 with linespoints lt 7 pt 25 ps 1.80 title "C3x", \
     "CubicSplineControlPoses.2.data.csv"  u  8:5 with linespoints lt 4 pt 25 ps 1.50 title "C2x", \
     "CubicSplineControlPoses.1.data.csv"  u  8:5 with linespoints lt 3 pt 25 ps 1.20 title "C1x", \
     "TargetKinFit.Pre4.data.csv" u 20:5 with linespoints lt 9 pt 23 ps 2.00 title "Pre4x", \
     "TargetKinFit.Pre3.data.csv" u 20:5 with linespoints lt 7 pt 23 ps 1.75 title "Pre3x", \
     "TargetKinFit.Pre2.data.csv" u 20:5 with linespoints lt 4 pt 23 ps 1.50 title "Pre2x", \
     "TargetKinFit.Pre1.data.csv" u 20:5 with linespoints lt 3 pt 23 ps 1.25 title "Pre1x", \
     "TargetKinFit.4.data.csv" u 20:5 with linespoints lt 9 pt 4 ps 2.00 title "4x", \
     "TargetKinFit.3.data.csv" u 20:5 with linespoints lt 7 pt 4 ps 1.80 title "3x", \
     "TargetKinFit.2.data.csv" u 20:5 with linespoints lt 4 pt 4 ps 1.50 title "2x", \
     "TargetKinFit.1.data.csv" u 20:5 with linespoints lt 3 pt 4 ps 1.20 title "1x"

