for BT in  scripts/onlyidbanditucb.sh scripts/onlyidrandom.sh scripts/onlyidbanditegrdy.sh scripts/onlyidreactive.sh
do
	for EXP in 1 2 3 4 5
	do
	     
	      ./$BT Evaluation2
	      SECONDS=0 
	      
	      echo 
	      while [ ! -f ~/rebet_ws/mission.done -a $SECONDS -lt 2520 ]
	      do
	      	  echo "waiting for mission to be done"              
		  sleep 10 #sustainability!
	      done
	      echo "mission done"
	      rm ~/rebet_ws/mission.done
	      ./scripts/killall.sh

	done
done

