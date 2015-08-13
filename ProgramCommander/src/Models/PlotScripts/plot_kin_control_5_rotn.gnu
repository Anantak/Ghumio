reset
set key maxrows 3 font ",6"
plot "TargetPoses.5.data.csv"  u  8:1 with linespoints lt 8 pt 6 ps 1.5 title "Tx", \
     "TargetPoses.5.data.csv"  u  8:2 with linespoints lt 8 pt 6 ps 1.5 title "Ty", \
     "TargetPoses.5.data.csv"  u  8:3 with linespoints lt 8 pt 6 ps 1.5 title "Tz", \
     "TargetPosesInterp.5.data.csv"  u  8:1 with linespoints lt 8 pt 9 title "Txi", \
     "TargetPosesInterp.5.data.csv"  u  8:2 with linespoints lt 8 pt 9 title "Tyi", \
     "TargetPosesInterp.5.data.csv"  u  8:3 with linespoints lt 8 pt 9 title "Tzi", \
     "CubicSplineStateControlPoses.5.data.csv"  u  8:1 with linespoints lt 7 pt 16 ps 1 title "SC5x", \
     "CubicSplineStateControlPoses.5.data.csv"  u  8:2 with linespoints lt 7 pt 16 ps 1 title "SC5y", \
     "CubicSplineStateControlPoses.5.data.csv"  u  8:3 with linespoints lt 7 pt 16 ps 1 title "SC5z", \
     "CubicSplineControlPoses.Pre5.data.csv"  u  8:1 with linespoints lt 1 pt 6 ps 1.40 title "PreC5x", \
     "CubicSplineControlPoses.Pre5.data.csv"  u  8:2 with linespoints lt 1 pt 6 ps 1.40 title "PreC5y", \
     "CubicSplineControlPoses.Pre5.data.csv"  u  8:3 with linespoints lt 1 pt 6 ps 1.40 title "PreC5z", \
     "CubicSplineControlPoses.5.data.csv"  u  8:1 with linespoints lt 1 pt 25 ps 1.20 title "C5x", \
     "CubicSplineControlPoses.5.data.csv"  u  8:2 with linespoints lt 1 pt 25 ps 1.20 title "C5y", \
     "CubicSplineControlPoses.5.data.csv"  u  8:3 with linespoints lt 1 pt 25 ps 1.20 title "C5z", \
     "TargetKinFit.Pre5.data.csv" u 20:1 with linespoints lt 3 pt 23 ps 1.25 title "Pre5x", \
     "TargetKinFit.Pre5.data.csv" u 20:2 with linespoints lt 3 pt 23 ps 1.25 title "Pre5y", \
     "TargetKinFit.Pre5.data.csv" u 20:3 with linespoints lt 3 pt 23 ps 1.25 title "Pre5z", \
     "TargetKinFit.5.data.csv" u 20:1 with linespoints lt 3 pt 4 ps 1.20 title "5x", \
     "TargetKinFit.5.data.csv" u 20:2 with linespoints lt 3 pt 4 ps 1.20 title "5y", \
     "TargetKinFit.5.data.csv" u 20:3 with linespoints lt 3 pt 4 ps 1.20 title "5z", \

