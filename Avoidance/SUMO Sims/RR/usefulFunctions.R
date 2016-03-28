timeToTravelDist <- function(vv){
  if(is.list(vv)){ vv = unlist(vv)}
  # vv = [distance, velocity, acceleration]
  sols = polyroot(vv * c(-1, 1, .5) )
  sols = sols[ abs(Im(sols)) < 10^-7 ] # don't count imaginary solutions
  if(length(sols) == 0){
    return( 10^10 )
  }
  sols = Re(sols)
  sols = sols[sols > 0] # don't count negative solutions
  if(length(sols) == 0){
    return( 10^10 )
  }
  return( min(sols) )
}

getFutureState <- function(vv, deltaT, opposite.dir = FALSE){
  if(is.list(vv)){ vv = unlist(vv)}
  if(opposite.dir){
    vv["Position"] = vv["Position"] - vv["Speed"]*deltaT - vv["Acceleration"]/2*deltaT^2
  } else{
    vv["Position"] = vv["Position"] + vv["Speed"]*deltaT + vv["Acceleration"]/2*deltaT^2
  }
  vv["Speed"] = vv["Speed"] + vv["Acceleration"]*deltaT
  return( vv )
}


### function to compute headways, given the data at t_rp
headways <- function(data){
  # find time where passing crosses lead
  vv = unlist(data["Passing",]) - unlist(data["Lead",])
  vv[1] = data["Lead","Position"] + VEH.length - data["Passing","Position"]
  t0Lead = timeToTravelDist(vv)
  # now compute headway with oncoming vehicle
  futurePassing = getFutureState(data["Passing",], t0Lead)
  futureOncoming = getFutureState(data["Oncoming",], t0Lead, opposite.dir=TRUE)
  if(futurePassing["Position"] > futureOncoming["Position"]){
    currentOncomingHeadway.0 = 0
  } else {
    vv = futurePassing + futureOncoming
    vv[1] = futureOncoming["Position"] - futurePassing["Position"]
    currentOncomingHeadway.0 = timeToTravelDist(vv)
  }
  
  # find time where passing has 1-s headway on lead
  futureLead = getFutureState(unlist(data["Lead",]), 1)
  vv = unlist(data["Passing",]) - futureLead
  vv[1] = futureLead["Position"] + VEH.length - data["Passing","Position"]
  t1Lead = timeToTravelDist(vv)
  #compute headway
  futurePassing = getFutureState(data["Passing",], t1Lead)
  futureOncoming = getFutureState(data["Oncoming",], t1Lead, opposite.dir=TRUE)
  if(futurePassing["Position"] > futureOncoming["Position"]){
    currentOncomingHeadway.1 = 0
  } else {
    vv = futurePassing + futureOncoming
    vv[1] = futureOncoming["Position"] - futurePassing["Position"]
    currentOncomingHeadway.1 = timeToTravelDist(vv)
  }
  
  # find time where passing crosses oncoming
  vv = unlist(data["Passing",]) + unlist(data["Oncoming",])
  vv[1] = data["Oncoming","Position"] - data["Passing","Position"]
  t0Oncoming = timeToTravelDist(vv)
  # compute lead headway at this time
  futurePassing = getFutureState(data["Passing",], t0Oncoming)
  futureLead = getFutureState(data["Lead",], t0Oncoming)
  if( futureLead["Position"] > futurePassing["Position"] - VEH.length){
    currentLeadHeadway.0 = 0
  } else {
    vv = futureLead
    vv["Position"] = futurePassing["Position"] - futureLead["Position"] - VEH.length
    currentLeadHeadway.0 = timeToTravelDist(vv)
  }
  
  # time where passing is 1s away from oncoming
  t1Oncoming = t0Oncoming - 1
  # compute headway
  futurePassing = getFutureState(data["Passing",], t1Oncoming)
  futureLead = getFutureState(data["Lead",], t1Oncoming)
  if( futureLead["Position"] > futurePassing["Position"] - VEH.length){
    currentLeadHeadway.1 = 0
  } else {
    vv = futureLead
    vv["Position"] = futurePassing["Position"] - futureLead["Position"] - VEH.length
    currentLeadHeadway.1 = timeToTravelDist(vv)
  }
  
  return( c(currentLeadHeadway.0, currentLeadHeadway.1, 
            currentOncomingHeadway.0, currentOncomingHeadway.1))
}


confusion <- function(truth, guess){
  ll = data.frame(actual.true = c(0,0),actual.false = c(0,0))
  ll[1,1] = sum((truth > 0)*(guess > 0), na.rm = TRUE)
  ll[2,1] = sum((truth > 0)*(guess == 0), na.rm = TRUE)
  ll[1,2] = sum((truth == 0)*(guess > 0), na.rm = TRUE)
  ll[2,2] = sum((truth == 0)*(guess == 0) , na.rm = TRUE)
  return(ll)
}


randomInRange <- function(param, range, bounds){
  if(length(bounds) == 1){
    minval = -6000
    maxval = 6000
    realrange = range * bounds
  } else {
    minval = bounds[1]
    maxval = bounds[2]
    realrange = range * (bounds[2] - bounds[1])
  }
  lowbound = max( minval, param - realrange)
  highbound = min( maxval, param + realrange)
  return( runif(1, lowbound, highbound) )
}


roc <- function(truth, headway){
  valsToCheck = rev(c(-1, 0, .1, .15, .2, .3, .4, .6, .8, 1.2, 1.6, 2.4, 3.2, 4.8, 6.4, 9.6, 12.8))
  TT = 0
  FF = 0
  for (val in valsToCheck){
    guess = headway > val
    conf = confusion(truth, guess)
    TT = c(TT, conf[1,1]/sum(conf[,1]))
    FF = c(FF, conf[1,2]/sum(conf[,2]))
  }
  plot(FF,TT)
  auc = 0
  for(i in 2:length(FF)){
    auc = auc + (TT[i] + TT[i-1])*(FF[i] - FF[i-1]) / 2
  }
  return(list(cutofff=valsToCheck, TT, FF, auc))
}


line.fit <- function(x,y){
  xNorm = x - mean(x)
  a = sum(xNorm * y) / sum(xNorm * x)
  b = sum(y - a * x) / length(x)
  R2 = 1 - sum((y - a * x - b)^2) / sum((y - mean(y))^2)
  return( c(a,b,R2) ) 
}


### just one headway
headways2 <- function(data){

  # find time where passing has 1-s headway on lead
  futureLead = getFutureState(unlist(data["Lead",]), 1)
  if(futureLead["Position"] < data["Passing","Position"]){
    t1Lead = 0
  } else {
    vv = unlist(data["Passing",]) - futureLead
    vv["Position"] = futureLead["Position"] + VEH.length - data["Passing","Position"]
    t1Lead = timeToTravelDist(vv)
  }
  #compute oncoming vehicle's headway (TTC)
  futurePassing = getFutureState(data["Passing",], t1Lead)
  futureOncoming = getFutureState(data["Oncoming",], t1Lead, opposite.dir=TRUE)
  if(futurePassing["Position"] > futureOncoming["Position"]){
    currentOncomingHeadway.1 = 0
  } else {
    vv = futurePassing + futureOncoming
    vv[1] = futureOncoming["Position"] - futurePassing["Position"]
    currentOncomingHeadway.1 = timeToTravelDist(vv)
  }

  return( currentOncomingHeadway.1 )
}


confusion <- function(truth, guess){
  ll = data.frame(actual.true = c(0,0),actual.false = c(0,0))
  ll[1,1] = sum((truth > 0)*(guess > 0), na.rm = TRUE)
  ll[2,1] = sum((truth > 0)*(guess == 0), na.rm = TRUE)
  ll[1,2] = sum((truth == 0)*(guess > 0), na.rm = TRUE)
  ll[2,2] = sum((truth == 0)*(guess == 0) , na.rm = TRUE)
  return(ll)
}


roc <- function(truth, headway){
  valsToCheck = rev(c(-1, 0, .1, .15, .2, .3, .4, .6, .8, 1.2, 1.6, 2.4, 3.2, 4.8, 6.4, 9.6, 12.8))
  TT = 0
  FF = 0
  for (val in valsToCheck){
    guess = headway > val
    conf = confusion(truth, guess)
    TT = c(TT, conf[1,1]/sum(conf[,1]))
    FF = c(FF, conf[1,2]/sum(conf[,2]))
  }
  plot(FF,TT)
  auc = 0
  for(i in 2:length(FF)){
    auc = auc + (TT[i] + TT[i-1])*(FF[i] - FF[i-1]) / 2
  }
  return(list(cutofff=valsToCheck, TT, FF, auc))
}


line.fit <- function(x,y){
  xNorm = x - mean(x)
  a = sum(xNorm * y) / sum(xNorm * x)
  b = sum(y - a * x) / length(x)
  R2 = 1 - sum((y - a * x - b)^2) / sum((y - mean(y))^2)
  return( c(a,b,R2) ) 
}