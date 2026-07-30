function bus = stateBus()
%Define the common vehicle-state interface.

names = ["X", "Y", "psi", "Vx", "Vy", "r", "beta", "ay"];
units = ["m", "m", "rad", "m/s", "m/s", "rad/s", "rad", "m/s^2"];

for k = numel(names):-1:1
    elements(k) = Simulink.BusElement;
    elements(k).Name = names(k);
    elements(k).DataType = "double";
    elements(k).Dimensions = 1;
    elements(k).Unit = units(k);
end

bus = Simulink.Bus;
bus.Description = "Common vehicle state.";
bus.Elements = elements;
end