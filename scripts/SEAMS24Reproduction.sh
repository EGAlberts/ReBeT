#Experiment1
HYP=("none" "['epsilon','0.2']" "['epsilon','1.0','decay_rate','1.09']")
BANDITS=("UCB" "egreedy" "egreedy")


ADAPPERIOD=16
DETTHRESH=14
for DETRATE in 1 3 5 7
do
      for EXPNUM in {1..30}
      do
            ./scripts/onlyid.sh EvaluationOne $ADAPPERIOD $ADAPPERIOD none none none $ADAPPERIOD $DETRATE $DETTHRESH
            SECONDS=0 
            while [ ! -f ~/rebet_ws/scripts/mission.done -a $SECONDS -lt 1680 ]
            do
                  echo "waiting for mission to be done" $EXP             
            sleep 10 #sustainability!
            done
            echo "mission done"
            rm ~/rebet_ws/scripts/mission.done
            ./scripts/killall.sh

      done
done



ADAPPERIOD=8
DETTHRESH=21

for BANDITNUM in 0 1 2
do
      for EXPNUM in {1..30}
      do
            ./scripts/onlyid.sh EvaluationTwo $ADAPPERIOD $ADAPPERIOD bandit ${BANDITS[BANDITNUM]} ${HYP[BANDITNUM]} $ADAPPERIOD none $DETTHRESH
            SECONDS=0 
            while [ ! -f ~/rebet_ws/scripts/mission.done -a $SECONDS -lt 2520 ]
            do
                  echo "waiting for mission to be done" $EXP             
            sleep 10 #sustainability!
            done
            echo "mission done"
            rm ~/rebet_ws/scripts/mission.done
            ./scripts/killall.sh

      done
done