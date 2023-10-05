for PIC in 1 3 5 7
do
	for EXP in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20
	do
	     
	      ./demo_onlyid.sh EvaluationOneProductBiasa $PIC onlyID.xml 14
	      SECONDS=0 
	      
	      echo 
	      while [ ! -f ~/rebet_ws/mission.done -a $SECONDS -lt 1680 ]
	      do
	      	  echo "waiting for mission to be done"              
		  sleep 10 #sustainability!
	      done
	      echo "mission done"
	      rm ~/rebet_ws/mission.done
	      ./killall.sh

	done
done
