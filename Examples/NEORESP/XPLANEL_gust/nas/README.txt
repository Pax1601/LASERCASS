File included:

XplaneL.nas: NASTRAN main file for GUST
XplaneL_nas.inc: NASTRAN aeroelastic model (included by the previous file)
XplaneL.f06: NASTRAN results
XplaneL.pch: NASTRAN results
%
XplaneL_ham.nas: NASTRAN alter to export Qhh
XplaneL_ham.f06: results from previous file
%
bending2002.png: comparison of bending response for BAR 2002
torque2002.png:  comparison of torque response for BAR 2002
shear2002.png: comparison of shear response for BAR 2002
q3_plunge.png: comparison of rigid plunge mode
q5_pitch.png: comparison of rigid pitch mode
q7_1stbend.png: comparison of first wing bending mode
The comparisons are between NASTRAN and XplaneL_neo.dat model

Hint: rigid shapes predicted by NASTRAN are not unitary displacements.
They hence have been scaled in q3_plunge.png and q5_plunge.png to
consistently compare with NEORESP