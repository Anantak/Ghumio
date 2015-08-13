reset
set key maxrows 3 font ",6"
plot "TargetPoses.2.data.csv"  u  8:1 with linespoints lt 8 pt 6 ps 1.5 title "Tx", \
     "TargetPoses.2.data.csv"  u  8:2 with linespoints lt 8 pt 6 ps 1.5 title "Ty", \
     "TargetPoses.2.data.csv"  u  8:3 with linespoints lt 8 pt 6 ps 1.5 title "Tz", \
     "TargetPosesInterp.2.data.csv"  u  8:1 with linespoints lt 8 pt 9 title "Txi", \
     "TargetPosesInterp.2.data.csv"  u  8:2 with linespoints lt 8 pt 9 title "Tyi", \
     "TargetPosesInterp.2.data.csv"  u  8:3 with linespoints lt 8 pt 9 title "Tzi", \
     "CubicSplineStateControlPoses.2.data.csv"  u  8:1 with linespoints lt 7 pt 16 ps 1 title "SC2x", \
     "CubicSplineStateControlPoses.2.data.csv"  u  8:2 with linespoints lt 7 pt 16 ps 1 title "SC2y", \
     "CubicSplineStateControlPoses.2.data.csv"  u  8:3 with linespoints lt 7 pt 16 ps 1 title "SC2z", \
     "CubicSplineControlPoses.Pre2.data.csv"  u  8:1 with linespoints lt 3 pt 6 ps 1.40 title "PreC2x", \
     "CubicSplineControlPoses.Pre2.data.csv"  u  8:2 with linespoints lt 3 pt 6 ps 1.40 title "PreC2y", \
     "CubicSplineControlPoses.Pre2.data.csv"  u  8:3 with linespoints lt 3 pt 6 ps 1.40 title "PreC2z", \
     "CubicSplineControlPoses.2.data.csv"  u  8:1 with linespoints lt 3 pt 25 ps 1.20 title "C2x", \
     "CubicSplineControlPoses.2.data.csv"  u  8:2 with linespoints lt 3 pt 25 ps 1.20 title "C2y", \
     "CubicSplineControlPoses.2.data.csv"  u  8:3 with linespoints lt 3 pt 25 ps 1.20 title "C2z", \
     "TargetKinFit.Pre2.data.csv" u 20:1 with linespoints lt 3 pt 23 ps 1.25 title "Pre2x", \
     "TargetKinFit.Pre2.data.csv" u 20:2 with linespoints lt 3 pt 23 ps 1.25 title "Pre2y", \
     "TargetKinFit.Pre2.data.csv" u 20:3 with linespoints lt 3 pt 23 ps 1.25 title "Pre2z", \
     "TargetKinFit.2.data.csv" u 20:1 with linespoints lt 3 pt 4 ps 1.20 title "2x", \
     "TargetKinFit.2.data.csv" u 20:2 with linespoints lt 3 pt 4 ps 1.20 title "2y", \
     "TargetKinFit.2.data.csv" u 20:3 with linespoints lt 3 pt 4 ps 1.20 title "2z", \

