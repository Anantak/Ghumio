reset
set key maxrows 6 font ",6"
plot "TargetPoses.0.data.csv"  u  8:5 with linespoints lt 8 pt 6 ps 1.5 title "Tx", \
     "TargetPosesInterp.0.data.csv"  u  8:5 with linespoints lt 8 pt 9 title "Txi", \
     "CubicSplineStateControlPoses.0.data.csv"  u  8:5 with linespoints lt 7 pt 28 ps 1 title "SC0x", \
     "CubicSplineControlPoses.Pre0.data.csv"  u  8:5 with linespoints lt 7 pt 28 ps 1.20 title "PreC0x", \
     "CubicSplineControlPoses.Pre9.data.csv"  u  8:5 with linespoints lt 4 pt 28 ps 1.00 title "PreC9x", \
     "CubicSplineControlPoses.Pre8.data.csv"  u  8:5 with linespoints lt 3 pt 28 ps 0.80 title "PreC8x", \
     "CubicSplineControlPoses.0.data.csv"  u  8:5 with linespoints lt 7 pt 25 ps 1.80 title "C0x", \
     "CubicSplineControlPoses.9.data.csv"  u  8:5 with linespoints lt 4 pt 25 ps 1.50 title "C9x", \
     "CubicSplineControlPoses.8.data.csv"  u  8:5 with linespoints lt 3 pt 25 ps 1.20 title "C8x", \
     "TargetKinFit.1.data.csv" u 20:5 with linespoints lt 1 pt 1 title "1x", \
     "TargetKinFit.2.data.csv" u 20:5 with linespoints lt 2 pt 1 title "2x", \
     "TargetKinFit.3.data.csv" u 20:5 with linespoints lt 1 pt 1 title "3x", \
     "TargetKinFit.4.data.csv" u 20:5 with linespoints lt 2 pt 1 title "4x", \
     "TargetKinFit.5.data.csv" u 20:5 with linespoints lt 1 pt 1 title "5x", \
     "TargetKinFit.6.data.csv" u 20:5 with linespoints lt 2 pt 1 title "6x", \
     "TargetKinFit.7.data.csv" u 20:5 with linespoints lt 1 pt 1 title "7x", \
     "TargetKinFit.Pre0.data.csv" u 20:5 with linespoints lt 7 pt 23 ps 1.2 title "Pre0x", \
     "TargetKinFit.Pre9.data.csv" u 20:5 with linespoints lt 4 pt 23 ps 1.2 title "Pre9x", \
     "TargetKinFit.Pre8.data.csv" u 20:5 with linespoints lt 3 pt 23 ps 1.2 title "Pre8x", \
     "TargetKinFit.0.data.csv" u 20:5 with linespoints lt 7 pt 5 title "0x", \
     "TargetKinFit.9.data.csv" u 20:5 with linespoints lt 4 pt 5 title "9x", \
     "TargetKinFit.8.data.csv" u 20:5 with linespoints lt 3 pt 5 title "8x"

