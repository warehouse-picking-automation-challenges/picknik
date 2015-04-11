function scanThisDirectoryForGit()
{
    for x in `find \`pwd\` -name .git -type d -prune`; do
	cd $x
	cd ../
	pwd
	git pull
    done
}

cd /home/$USER/ros/ws_amazon/src
#cd /home/$USER/ros/ws_picknik/src
scanThisDirectoryForGit

echo ""
echo "Finished pulling from all ROS repos!"
echo ""

#alias gitpullall="source ~/ros/ws_picknik/src/picknik/git_pull_all_repos.sh"
