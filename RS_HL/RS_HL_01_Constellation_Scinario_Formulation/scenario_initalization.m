function scenario = scenario_initalization(time_parameters)

startTime = time_parameters.startTime;
duration = time_parameters.duration;
steptime = time_parameters.steptime;

stopTime = startTime + duration;
scenario = satelliteScenario(startTime,stopTime,steptime);

end