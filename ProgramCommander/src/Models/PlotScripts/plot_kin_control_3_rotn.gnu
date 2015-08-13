reset
set key maxrows 3 font ",6"
plot "TargetPoses.3.data.csv"  u  8:1 with linespoints lt 8 pt 6 ps 1.5 title "Tx", \
     "TargetPoses.3.data.csv"  u  8:2 with linespoints lt 8 pt 6 ps 1.5 title "Ty", \
     "TargetPoses.3.data.csv"  u  8:3 with linespoints lt 8 pt 6 ps 1.5 title "Tz", \
     "TargetPosesInterp.3.data.csv"  u  8:1 with linespoints lt 8 pt 9 title "Txi", \
     "TargetPosesInterp.3.data.csv"  u  8:2 with linespoints lt 8 pt 9 title "Tyi", \
     "TargetPosesInterp.3.data.csv"  u  8:3 with linespoints lt 8 pt 9 title "Tzi", \
     "CubicSplineStateControlPoses.3.data.csv"  u  8:1 with linespoints lt 7 pt 16 ps 1 title "SC3x", \
     "CubicSplineStateControlPoses.3.data.csv"  u  8:2 with linespoints lt 7 pt 16 ps 1 title "SC3y", \
     "CubicSplineStateControlPoses.3.data.csv"  u  8:3 with linespoints lt 7 pt 16 ps 1 title "SC3z", \
     "CubicSplineControlPoses.Pre3.data.csv"  u  8:1 with linespoints lt 1 pt 6 ps 1.40 title "PreC3x", \
     "CubicSplineControlPoses.Pre3.data.csv"  u  8:2 with linespoints lt 1 pt 6 ps 1.40 title "PreC3y", \
     "CubicSplineControlPoses.Pre3.data.csv"  u  8:3 with linespoints lt 1 pt 6 ps 1.40 title "PreC3z", \
     "CubicSplineControlPoses.3.data.csv"  u  8:1 with linespoints lt 1 pt 25 ps 1.20 title "C3x", \
     "CubicSplineControlPoses.3.data.csv"  u  8:2 with linespoints lt 1 pt 25 ps 1.20 title "C3y", \
     "CubicSplineControlPoses.3.data.csv"  u  8:3 with linespoints lt 1 pt 25 ps 1.20 title "C3z", \
     "TargetKinFit.Pre3.data.csv" u 20:1 with linespoints lt 3 pt 23 ps 1.25 title "Pre3x", \
     "TargetKinFit.Pre3.data.csv" u 20:2 with linespoints lt 3 pt 23 ps 1.25 title "Pre3y", \
     "TargetKinFit.Pre3.data.csv" u 20:3 with linespoints lt 3 pt 23 ps 1.25 title "Pre3z", \
     "TargetKinFit.3.data.csv" u 20:1 with linespoints lt 3 pt 4 ps 1.20 title "3x", \
     "TargetKinFit.3.data.csv" u 20:2 with linespoints lt 3 pt 4 ps 1.20 title "3y", \
     "TargetKinFit.3.data.csv" u 20:3 with linespoints lt 3 pt 4 ps 1.20 title "3z", \

