# I apologize for the mess this currently is, I will add more comments soon
# last modified 3/28/16
## you have to set your own file location here:
#setwd("C:/Users/m2/Documents/GradLab/carstop/Avoidance/SUMO Sims/RR")
setwd("/home/motrom/Documents/carstop/Avoidance/SUMO Sims/RR")

#for(PARAM.loss.rate in c(0.,.5)){
number.of.files = 5000
PARAM.loss.rate = 0.   # ratio of communication packets lost (0 to 1)  [0, .33, .67]
PARAM.randomization = 1.  # total range of (uniform) noise applied to parameters [0, .25, .5, 1]
PARAM.estimationNoise = PARAM.randomization
#PARAM.sensor = 'V2V'
PARAM.sensor = 'V2Ilong_500_1100'

VEH.overtake.accel.bounds = c(.305, 2.5)
VEH.trp.bounds = c(1, 4)
VEH.sensed.position.range = .5 # 4
VEH.sensed.speed.range = .5 # 1.5
VEH.sensed.acceleration.range = .25 # 
VEH.length = 5.8
VEH.prep.time = 0.0
allParameters <- read.csv("../Parameters/RR_Parameters.csv")#"results/params.csv")
allParameters["Perception.Response.Time"] = 
  allParameters["Perception.Response.Time"] - VEH.prep.time
if (nrow(allParameters) > number.of.files){
    allParameters = allParameters[1:number.of.files,] }
source("usefulFunctions.R")

# setting up data frames
valid = commFailed = logical(number.of.files)
#commDist = numeric(number.of.files)
headway.actual = headway.trp = numeric(number.of.files)
min.warning.time = numeric(number.of.files)
passbackTimes = numeric(number.of.files) # neww

for(file.num in 1:number.of.files){
  
#file.num = 5
parameters = unlist(allParameters[file.num,])
trp = parameters["Perception.Response.Time"]
#data <- read.csv(paste("results/simData",file.num,".csv", sep=""), header=TRUE)
#data = data[, c(1,2,4,7,8)] #Time, Vehicle.ID, Position.x, Speed, Acceleration
#names(data)[3] = "Position"
data <- read.csv(paste("../Results/RR/",file.num,".csv", sep=""), header=TRUE)
data = data[,c(1,2,3,6,7)] # time, vehID, x, speed, Acceleration
names(data) = c("Time","Vehicle.ID","Position","Speed","Acceleration")

if(nrow(data)>0 && ncol(data)>0){
  data["Time"] = data["Time"] - VEH.prep.time
  data = data[data["Time"]>=0,]
}

## sort into timeblocks
time.sorted.data = NULL
list.of.sim.times = NULL
current.data = NULL
time.index = data[1,"Time"]
index = 1
while (index < nrow(data)) {
  current.data = rbind(current.data, data[index,])
  index = index + 1
  if((index > nrow(data)) || (data[index,"Time"] > time.index)){
    row.names(current.data) = current.data[,"Vehicle.ID"]
    current.data = current.data[,-c(1,2)] # don't store time or ID as columns
    if( nrow(current.data) == 3 ){  #don't include timesteps without all vehicles
      time.sorted.data = c(time.sorted.data, list(current.data))
      list.of.sim.times = c(list.of.sim.times, time.index)
    }
    time.index = data[index,"Time"]
    current.data = NULL
  }
}


## run checks on data - eliminate it if the scenario is implausible or not useful
invalid = FALSE
# check 1:  passing vehicle never passes lead (before at least one player exits)
final.time.data = time.sorted.data[[length(time.sorted.data)]]
invalid = invalid || (final.time.data["Lead","Position"] >
                              final.time.data["Passing","Position"] - VEH.length)

# check 2:  at t_rp, oncoming and lead vehicles have already crossed
trp.index = sum(list.of.sim.times < trp) + 1
trp.index = min(trp.index, length(time.sorted.data))
trp.time.data = time.sorted.data[[trp.index]]
#commDist[file.num + 1] = trp.time.data["Oncoming","Position"] - trp.time.data["Passing","Position"]
invalid = invalid || (trp.time.data["Oncoming","Position"] < trp.time.data["Lead","Position"])

# check 3:  at t_rp, passing and lead vehicles have already collided
invalid = invalid || (trp.time.data["Lead","Position"]  - VEH.length <
                              trp.time.data["Passing","Position"])

if(is.na(invalid)){
  invalid = TRUE
}

valid[file.num] = !invalid

if(invalid){
  headway.actual[file.num] = NA
  headway.trp[file.num] = NA
  min.warning.time[file.num] = NA
  passbackTimes[file.num] = NA  # neww
}
else{
  
## search for head-time measures
oncomingHeadway = NULL
counter = 0
passbackTime = -1
for(current.data in time.sorted.data){
  counter = counter + 1
  # lead headway, 0 if passing vehicle is not actually ahead yet
  if( current.data["Lead","Position"] >= current.data["Passing","Position"] - VEH.length){
    currentLeadHeadway = 0
  } else {
    vv = unlist(current.data["Lead",])
    vv["Position"] = current.data["Passing","Position"] - VEH.length - vv["Position"]
    currentLeadHeadway = timeToTravelDist(vv)
  }
  if (is.null(oncomingHeadway) && currentLeadHeadway >= 1){
    passbackTime = list.of.sim.times[counter]
    if(current.data["Passing","Position"] > current.data["Oncoming","Position"]){
      oncomingHeadway = 0
    } else {
      # shift a little to get to actual headway=1 position
      vv = unlist(current.data["Passing",] - current.data["Lead",])
      vv["Position"] = current.data["Lead","Speed"]*(currentLeadHeadway - 1) + current.data["Lead","Acceleration"]*.06
      if(vv["Position"] > .01){
        timecorrection = -timeToTravelDist(vv)
      } else{
        timecorrection = 0
      }
      current.data["Passing",] = getFutureState(current.data["Passing",],timecorrection)
      current.data["Oncoming",] = getFutureState(current.data["Oncoming",],timecorrection, opposite.dir = TRUE)
      # solve for oncoming headway (TTC)
      vv = unlist(current.data["Passing",]) + unlist(current.data["Oncoming",])
      vv["Position"] = current.data["Oncoming","Position"] - current.data["Passing","Position"]
      oncomingHeadway = timeToTravelDist(vv)
    }
  }
}
if(is.null(oncomingHeadway)){ oncomingHeadway=0}
headway.actual[file.num] = oncomingHeadway
passbackTimes[file.num] = passbackTime

if(TRUE){
## now analyze communications file
#data <- read.csv(paste("results/commData",file.num,".csv", sep=""), header=TRUE)
#data = data[data["Receiver.ID"] == "Passing",] # only need Passing vehicle's information
#data = data[, c(11,2,4,7,8,1)] #Time.Received, Vehicle.ID, Position.x, Speed, Acceleration, Time.Sent
#names(data)[1] = "Time"
#names(data)[3] = "Position"
data <- read.csv(paste("../Sensor Results/RR/",PARAM.sensor,"/",file.num,
                        ".csv", sep=""), header=TRUE)
data["Time.Sent"] = data["Time.Sent"] - VEH.prep.time
data["Time"] = data["Time"] - VEH.prep.time
data = data[data["Time.Sent"]>=0,]

extraRow = data[1,]
extraRow["Time"] = 100

if(PARAM.loss.rate > 0){
  packets.not.lost = runif(nrow(data)) > PARAM.loss.rate
  data = data[packets.not.lost,]
}

for(rowi in 1:nrow(data)){
  data[rowi,"Position"] = randomInRange(data[rowi,"Position"],
                                        PARAM.randomization, VEH.sensed.position.range)
  data[rowi,"Speed"] = randomInRange(data[rowi,"Speed"],
                                     PARAM.randomization, VEH.sensed.speed.range)
  data[rowi,"Acceleration"] = randomInRange(data[rowi,"Acceleration"],
                                            PARAM.randomization, VEH.sensed.acceleration.range)
}

if(nrow(data)>0){
leadCommData = data[data["Sender.ID"] == "Lead",]
oncomingCommData = data[data["Sender.ID"] == "Oncoming",]
} else {
  leadCommData = data
  oncomingCommData = data
}

# I don't know why I was rounding these before...
#if (sum(leadCommData["Time"] < round(trp-.04999,1)) == 0){
#  commFailed[file.num]=TRUE
#}
#if (sum(oncomingCommData["Time"] < round(trp-.04999,1)) == 0){
#  commFailed[file.num]=TRUE
#}
if (nrow(leadCommData)==0 || sum(leadCommData["Time"] < trp) == 0){
  commFailed[file.num]=TRUE
}
if (nrow(oncomingCommData)==0 || sum(oncomingCommData["Time"] < trp) == 0){
  commFailed[file.num]=TRUE
}

if(commFailed[file.num]){
  headway.trp[file.num] = 100
  min.warning.time[file.num] = 0
} else {
  
# for cases where a vehicle has not been detected,
# sometimes must assume there is one outside of detectable range
nullVehicle = data.frame(Position = 5500, Speed = 0, Acceleration = 0, timer = trp)
passingAccelGuess = randomInRange(parameters["Overtaking.Acceleration"],
                                                   PARAM.estimationNoise, VEH.overtake.accel.bounds)
 
# take measurements nearest to t_rp
trp.data = time.sorted.data[[sum(list.of.sim.times < trp)]] # get own information from sim data
trp.data[,"timer"] = rep(list.of.sim.times[sum(list.of.sim.times < trp)],3)
if(sum(leadCommData["Time"] < trp) == 0){
  trp.data["Lead",] = nullVehicle
  trp.data["Lead","Position"] = trp.data["Passing","Position"] # guaranteed to pass quickly
  trp.data["Oncoming",] = nullVehicle # Jan edit, just make it impossible to not be safe
} else{
  trp.data["Lead",] = as.numeric(leadCommData[sum(leadCommData["Time"] < trp),3:6])
}
if(sum(oncomingCommData["Time"] < trp) == 0){   # didn't hear anything from oncoming
  trp.data["Oncoming",] = nullVehicle
} else{
  trp.data["Oncoming",] = as.numeric(oncomingCommData[sum(oncomingCommData["Time"] < trp),3:6])
}
# add randomness
#for(veh in 1:3){
#  trp.data[veh,"Position"] = randomInRange(trp.data[veh,"Position"],
#                                               PARAM.randomization, VEH.sensed.position.range)
#  trp.data[veh,"Speed"] = randomInRange(trp.data[veh,"Speed"],
#                                            PARAM.randomization, VEH.sensed.speed.range)
#  trp.data[veh,"Acceleration"] = randomInRange(trp.data[veh,"Acceleration"],
#                                                   PARAM.randomization, VEH.sensed.acceleration.range)
#}
trp.data["Passing","Acceleration"] = 0 # should be fine assuming this is perfectly known 
# move in case some of your data is from way back
  trp.data["Lead",] = getFutureState(trp.data["Lead",], trp - trp.data["Lead","timer"])
  trp.data["Passing",] = getFutureState(trp.data["Passing",], trp - trp.data["Passing","timer"])
  trp.data["Oncoming",] = getFutureState(trp.data["Oncoming",], trp - trp.data["Oncoming","timer"],
                                             opposite.dir = TRUE)
trp.data["Passing","Acceleration"] = passingAccelGuess
headway.trp[file.num] = headways2(trp.data[,-4])

# now check various timesteps
guessedTrp = randomInRange(trp, PARAM.estimationNoise, VEH.trp.bounds)
timestepOfWarning = 0.1
currentTime = 0.1
warningTime = 0
while((currentTime < trp) && (warningTime <= 0)){
  guessedTrp = max(guessedTrp, currentTime) # system should be smart enough to recognize when it's passed its guess
  current.data = time.sorted.data[[sum(list.of.sim.times < currentTime)]] # get own information from sim data
  current.data[,"timer"] = rep(list.of.sim.times[sum(list.of.sim.times < currentTime)],3)
  #info.times = unlist(list( Lead = 0, Passing = 0, Oncoming = 0 ))
  if(sum(leadCommData["Time"] < currentTime) == 0){
    current.data["Lead",] = nullVehicle
    current.data["Lead","Position"] = current.data["Passing","Position"]
    current.data["Oncoming",] = nullVehicle
  } else{
    current.data["Lead",] = as.numeric(leadCommData[sum(leadCommData["Time"] < currentTime),3:6])
    #info.times["Lead"] = leadCommData
  }
  if(sum(oncomingCommData["Time"] < currentTime) == 0){   # didn't hear anything from oncoming
    current.data["Oncoming",] = nullVehicle
  } else{
    current.data["Oncoming",] = as.numeric(oncomingCommData[sum(oncomingCommData["Time"] < currentTime),3:6])
  }
  # add randomness
  #for(veh in 1:3){
  #  current.data[veh,"Position"] = randomInRange(current.data[veh,"Position"],
  #                                               PARAM.randomization, VEH.sensed.position.range)
  #  current.data[veh,"Speed"] = randomInRange(current.data[veh,"Speed"],
  #                                               PARAM.randomization, VEH.sensed.speed.range)
  #  current.data[veh,"Acceleration"] = randomInRange(current.data[veh,"Acceleration"],
  #                                               PARAM.randomization, VEH.sensed.acceleration.range)
  #}
  current.data["Passing","Acceleration"] = 0 # should be fine assuming this is perfectly known 
  current.data["Lead",] = getFutureState(current.data["Lead",], guessedTrp - current.data["Lead","timer"])
  current.data["Passing",] = getFutureState(current.data["Passing",], guessedTrp - current.data["Passing","timer"])
  current.data["Oncoming",] = getFutureState(current.data["Oncoming",], guessedTrp - current.data["Oncoming","timer"],
                                               opposite.dir = TRUE)
  # finally, guess overtaking acceleration of passing vehicle
  current.data["Passing","Acceleration"] = passingAccelGuess
  # warn if computed headway is less than 1
  if(headways2(current.data[,-4]) < 1){
    warningTime = currentTime
  }
  currentTime = currentTime + timestepOfWarning
}
min.warning.time[file.num] = warningTime

}
}
}
}


### set up data table
collision = headway.actual < 1
was.warning = min.warning.time > 0
truth = data.frame(headway.actual, collision)
names(truth) = c("True Oncoming Headway when Lead Headway is 1s",
                   "Collision Occurred")
results = data.frame(!commFailed, headway.trp, min.warning.time, was.warning)
names(results) = c("Communication Established before t_rp",
                   "Estimate at t_rp of Oncoming Headway when Lead Headway is 1s",
                   "Time of Earliest Warning or 0 if no warning",
                   "Warning Occurred before t_rp")

#allParameters["Passing to Lead Distance at t0"] = (allParameters["Lead.Vehicle.Position.at.t0"] -
#                                                   allParameters["Passing.Vehicle.Position"])
#allParameters["Passing to Oncoming Distance at t0"] = (1300 - allParameters["Oncoming.Vehicle.Position.at.t0"] -
#                                                     allParameters["Passing.Vehicle.Position.at.t0"])
#allParameters = subset(allParameters, select=-c(Random.Seed, Passing.Vehicle.Position.at.t0,
#                                                Lead.Vehicle.Position.at.t0, Oncoming.Vehicle.Position.at.t0,
#                                                Passing.Vehicle.Acceleration.at.t0))
names(allParameters)[names(allParameters)=="Lead Vehicle Distance"] = "Passing to Lead Distance at t0"
names(allParameters)[names(allParameters)=="Oncoming Vehicle Distance"] = "Passing to Oncoming Distance at t0"

#Communication.Message.Interval.in.ms = rep(100, number.of.files)
#Communication.Power.in.mW = rep(75, number.of.files)
#Synthetic.Packet.Loss.Rate = rep(PARAM.loss.rate, number.of.files)
#Accuracy.of.Parameter.Estimation = rep(PARAM.randomization, number.of.files)
#newParams = data.frame(Communication.Message.Interval.in.ms, Communication.Power.in.mW,
#                       Synthetic.Packet.Loss.Rate, Accuracy.of.Parameter.Estimation)

DataTable = data.frame(allParameters, truth, results)
DataTable = DataTable[valid,]
DataTable = DataTable * 1  ## turn all booleans to 0/1
names(DataTable) <- gsub("\\.", " ", names(DataTable))
row.names(DataTable) = NULL

# save in file
csvname = paste("../Analysis/RR/",PARAM.sensor,"_sensors",PARAM.randomization,
                "_estimation",PARAM.estimationNoise,".csv",sep="")
#csvname = paste("../Analysis/RR/",PARAM.sensor,"_loss",PARAM.loss.rate,".csv",sep="")
write.csv(DataTable, file=csvname, row.names = FALSE)
#}