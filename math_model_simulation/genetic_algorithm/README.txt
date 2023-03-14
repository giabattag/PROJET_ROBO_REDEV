This document discuss about how to launch the GA, since the integration is made by hand qand not automatically

1 - You need to decouple the main from the initFnc of the simulink model;
otherwise it will always use the standart valeus of the controller

2- you need to regulate the gains W in the GeneticObj to satisfy your demand

3- The J funciont return the cost and the fitness of that object; You can change the calculation
changing this function