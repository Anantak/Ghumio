reset
set key maxrows 3 font ",6"
plot "TargetPoses.4.data.csv"  u  8:1 with linespoints lt 8 pt 6 ps 1.5 title "Tx", \
     "TargetPoses.4.data.csv"  u  8:2 with linespoints lt 8 pt 6 ps 1.5 title "Ty", \
     "TargetPoses.4.data.csv"  u  8:3 with linespoints lt 8 pt 6 ps 1.5 title "Tz", \
     "TargetPosesInterp.4.data.csv"  u  8:1 with linespoints lt 8 pt 9 title "Txi", \
     "TargetPosesInterp.4.data.csv"  u  8:2 with linespoints lt 8 pt 9 title "Tyi", \
     "TargetPosesInterp.4.data.csv"  u  8:3 with linespoints lt 8 pt 9 title "Tzi", \
     "CubicSplineStateControlPoses.4.data.csv"  u  8:1 with linespoints lt 7 pt 16 ps 1 title "SC4x", \
     "CubicSplineStateControlPoses.4.data.csv"  u  8:2 with linespoints lt 7 pt 16 ps 1 title "SC4y", \
     "CubicSplineStateControlPoses.4.data.csv"  u  8:3 with linespoints lt 7 pt 16 ps 1 title "SC4z", \
     "CubicSplineControlPoses.Pre4.data.csv"  u  8:1 with linespoints lt 1 pt 6 ps 1.40 title "PreC4x", \
     "CubicSplineControlPoses.Pre4.data.csv"  u  8:2 with linespoints lt 1 pt 6 ps 1.40 title "PreC4y", \
     "CubicSplineControlPoses.Pre4.data.csv"  u  8:3 with linespoints lt 1 pt 6 ps 1.40 title "PreC4z", \
     "CubicSplineControlPoses.4.data.csv"  u  8:1 with linespoints lt 1 pt 25 ps 1.20 title "C4x", \
     "CubicSplineControlPoses.4.data.csv"  u  8:2 with linespoints lt 1 pt 25 ps 1.20 title "C4y", \
     "CubicSplineControlPoses.4.data.csv"  u  8:3 with linespoints lt 1 pt 25 ps 1.20 title "C4z", \
     "TargetKinFit.Pre4.data.csv" u 20:1 with linespoints lt 3 pt 23 ps 1.25 title "Pre4x", \
     "TargetKinFit.Pre4.data.csv" u 20:2 with linespoints lt 3 pt 23 ps 1.25 title "Pre4y", \
     "TargetKinFit.Pre4.data.csv" u 20:3 with linespoints lt 3 pt 23 ps 1.25 title "Pre4z", \
     "TargetKinFit.4.data.csv" u 20:1 with linespoints lt 3 pt 4 ps 1.20 title "4x", \
     "TargetKinFit.4.data.csv" u 20:2 with linespoints lt 3 pt 4 ps 1.20 title "4y", \
     "TargetKinFit.4.data.csv" u 20:3 with linespoints lt 3 pt 4 ps 1.20 title "4z", \

