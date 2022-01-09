# Outlander-Charger-DCDC-DUE-code-
Code for controlling Outlander charger/DCDC in my Mazda

I rewired Mazda MX3 with Outlander charger which is also DCDC in one box. To run DCDC one needs only 12V enable signal, while for charger one needs CP signal as well as CAN command and heartbeat signal.
 DUE is observing CAN traffic on DCDC status report (this is why i keep it connected) and decides to switch DCDC to enable when 12V aux battery drops to 12.2V! That way i dont need to separately switch on the DCDC when charging. It comes online itself when needed.  
