    function chord = tailchord(taper_kink,taper_tip,root_chord,ref_span,span_kink)
    %Created by Javier Muñoz in May 2010 in KTH University under
    %supervision of Arthur Rizzi
    %This function computes the local chord length at given span station
    %compute chord on actual planform geometry for HTAIL, VTAIL and CANARD.
    %The purpose is to avoid the mistakes made by the general function
    %qxcdcop.
    %It is only used during the center of gravity estimations.
    %Input parameters:
%     taper_kink = chord at the kink divided by root chord.
%     taper_tip = chord at the tip divided by root chord(Taper ratio).
%     root_chord
%     ref_span = Non dimensional distance from root to the point where the chord is calculated.
%     span_kink = Non dimensional distance form the root to the chord.

    if ref_span > span_kink
        chord = (taper_tip-taper_kink)*(ref_span-span_kink)*root_chord + taper_kink*root_chord;
    else
        chord = (taper_kink-1)*root_chord*ref_span + root_chord;
    end
  