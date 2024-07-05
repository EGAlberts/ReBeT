# Kill all ros nodes that may be running
for i in $(ps au | grep -w 'ros' | grep -v 'ros2cli' | grep -v 'vscode' | grep -v 'grep' | awk '{print $2}')
do
    echo "kill -2 $i"
    kill -2 $i;
done
sleep 1
for i in $(ps au | grep -w 'ros' | grep -v 'ros2cli' | grep -v 'vscode' | grep -v 'grep' | awk '{print $2}')
do
    echo "kill -9 $i"
    kill -9 $i;
done
sleep 1
for i in $(ps -aux | grep suave_reasoner | grep -v /ros/ | grep -v grep | awk '{print $2}')
do
    echo "kill -2 $i"
    kill -2 $i;
done
sleep 1
for i in $(ps -aux | grep 'gz' | grep -v grep | awk '{print $2}')
do
    echo "kill -2 $i"
    kill -2 $i;
done
sleep 1
for i in $(ps -aux | grep mav | grep -v grep | awk '{print $2}')
do
    echo "kill -2 $i"
    kill -2 $i;
done
sleep 1
for i in $(ps -aux | grep ardu | grep -v grep | awk '{print $2}')
do
    echo "kill -2 $i"
    kill -2 $i;
done
sleep 10 #let it finish the killing spree


