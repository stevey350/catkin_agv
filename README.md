# catkin_agv

0.1.0 (2018-10-19)
------------------
* none

0.1.1 (2018-10-19)
------------------
* setup can0_config.sh auto-start script
* refactoring for release
* Contributors: Hong Shaojiang




1.将该sh文件移动到/etc/init.d/目录下，并修改权限

sudo cp can0_config.sh /etc/init.d
sudo chmod 755 /etc/init.d/can0_config.sh

3.设置启动顺序

cd /etc/init.d
sudo update-rc.d can0_config.sh defaults 95

其中数字95是脚本启动的顺序号，按照自己的需要相应修改即可。在你有多个启动脚本，而它们之间又有先后启动的依赖关系时你就知道这个数字的具体作用了

#卸载启动脚本
#cd /etc/init.d
#sudo update-rc.d -f mysetup.sh remove
