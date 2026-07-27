function bus = referenceBus()
%Define the common vehicle-state interface.

names = ["X_ref", "Y_ref", "psi_ref", "kappa_ref", "Vx_ref", "mu"];
units = ["m", "m", "rad", "1/m", "m/s", "1"];

for k = numel(names):-1:1
    elements(k) = Simulink.BusElement;
    elements(k).Name = names(k);
    elements(k).DataType = "double";
    elements(k).Dimensions = 1;
    elements(k).Unit = units(k);
end

bus = Simulink.Bus;
bus.Description = "Common references.";
bus.Elements = elements;
end