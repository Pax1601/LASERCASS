Files included:

B747-100.xml: main XML file with geometry data and settings
B747-100_CAS25.inc: files with maneuvers set through NEOCASS CAS panel 
(see B747-100_CAS25.png for the input values given)
B747-100_MTOW.inc: files with aeroelastic model
B747-100_MTOW_CONM.inc: payload to have MTOW configuration
B747-100_MTOW_guess.txt: GUESS outoput file with mass summary

Hint: to have MTOW value, the sizing process has been carried out through
GUESS elastic, setting a user-defined configuration with wing tank percentage of 40%.

Warning: the value for MTOW given in B747-100_MTOW_guess.txt considers max fuel 
which is not realistic when max passengers are loaded.
