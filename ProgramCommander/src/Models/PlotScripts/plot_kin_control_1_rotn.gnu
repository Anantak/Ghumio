reset
set key maxrows 3 font ",6"
plot "TargetPoses.1.data.csv"  u  8:1 with linespoints lt 8 pt 6 ps 1.5 title "Tx", \
     "TargetPoses.1.data.csv"  u  8:2 with linespoints lt 8 pt 6 ps 1.5 title "Ty", \
     "TargetPoses.1.data.csv"  u  8:3 with linespoints lt 8 pt 6 ps 1.5 title "Tz", \
     "TargetPosesInterp.1.data.csv"  u  8:1 with linespoints lt 8 pt 9 title "Txi", \
     "TargetPosesInterp.1.data.csv"  u  8:2 with linespoints lt 8 pt 9 title "Tyi", \
     "TargetPosesInterp.1.data.csv"  u  8:3 with linespoints lt 8 pt 9 title "Tzi", \
     "CubicSplineStateControlPoses.1.data.csv"  u  8:1 with linespoints lt 7 pt 16 ps 1 title "SC4x", \
     "CubicSplineStateControlPoses.1.data.csv"  u  8:2 with linespoints lt 7 pt 16 ps 1 title "SC4y", \
     "CubicSplineStateControlPoses.1.data.csv"  u  8:3 with linespoints lt 7 pt 16 ps 1 title "SC4z", \
     "CubicSplineControlPoses.Pre1.data.csv"  u  8:1 with linespoints lt 3 pt 6 ps 1.40 title "PreC1x", \
     "CubicSplineControlPoses.Pre1.data.csv"  u  8:2 with linespoints lt 3 pt 6 ps 1.40 title "PreC1y", \
     "CubicSplineControlPoses.Pre1.data.csv"  u  8:3 with linespoints lt 3 pt 6 ps 1.40 title "PreC1z", \
     "CubicSplineControlPoses.1.data.csv"  u  8:1 with linespoints lt 3 pt 25 ps 1.20 title "C1x", \
     "CubicSplineControlPoses.1.data.csv"  u  8:2 with linespoints lt 3 pt 25 ps 1.20 title "C1y", \
     "CubicSplineControlPoses.1.data.csv"  u  8:3 with linespoints lt 3 pt 25 ps 1.20 title "C1z", \
     "TargetKinFit.Pre1.data.csv" u 20:1 with linespoints lt 3 pt 23 ps 1.25 title "Pre1x", \
     "TargetKinFit.Pre1.data.csv" u 20:2 with linespoints lt 3 pt 23 ps 1.25 title "Pre1y", \
     "TargetKinFit.Pre1.data.csv" u 20:3 with linespoints lt 3 pt 23 ps 1.25 title "Pre1z", \
     "TargetKinFit.1.data.csv" u 20:1 with linespoints lt 3 pt 4 ps 1.20 title "1x", \
     "TargetKinFit.1.data.csv" u 20:2 with linespoints lt 3 pt 4 ps 1.20 title "1y", \
     "TargetKinFit.1.data.csv" u 20:3 with linespoints lt 3 pt 4 ps 1.20 title "1z", \

