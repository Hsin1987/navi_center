from geometry_msgs.msg import *
from move_base_msgs.msg import *

class GoalMsg:

    def __init__(self):
        print '[nc] HotelGoal inited'

    def toGoal(self, goal):
        goalDic = {'Lobby': self.toLobby,
                    '101':self.to101,'102':self.to102,
                    '100':self.to100, '199':self.to199, '198':self.to198,
                    'EVW1':self.toEVW1,'EVW2':self.toEVW2,'EVW3':self.toEVW3,'EVW4':self.toEVW4,
                    'EVW5':self.toEVW5,'EVW6':self.toEVW6,'EVW7':self.toEVW7,'EVW8':self.toEVW8,
                    'EVW1S':self.toEVW1S,'EVW2S':self.toEVW2S,'EVW3S':self.toEVW3S,'EVW4S':self.toEVW4S,
                    'EVW5S':self.toEVW5S,'EVW6S':self.toEVW6S,'EVW7S':self.toEVW7S,'EVW8S':self.toEVW8S,
                    '201':self.to201,'202':self.to202,'203':self.to203,'204':self.to204,'205':self.to205,
                    '206':self.to206,'207':self.to207,'208':self.to208,'209':self.to209,'210':self.to210,
                    '211':self.to211,'212':self.to212,'213':self.to213,
                    '301':self.to301,'302':self.to302,'303':self.to303,'304':self.to304,'305':self.to305,
                    '306':self.to306,'307':self.to307,'308':self.to308,'309':self.to309,'310':self.to310,
                    '311':self.to311,'312':self.to312,'313':self.to313,'314':self.to314,'315':self.to315,
                    '316':self.to316,'317':self.to317,'318':self.to318,'319':self.to319,'320':self.to320,
                    '321':self.to321,'322':self.to322,
                    '401':self.to401,'402':self.to402,'403':self.to403,'404':self.to404,'405':self.to405,
                    '406':self.to406,'407':self.to407,'408':self.to408,'409':self.to409,'410':self.to410,
                    '411':self.to411,'412':self.to412,'413':self.to413,'414':self.to414,'415':self.to415,
                    '416':self.to416,'417':self.to417,'418':self.to418,'419':self.to419,'420':self.to420,
                    '421':self.to421,'422':self.to422,
                    '501':self.to501,'502':self.to502,'503':self.to503,'504':self.to504,'505':self.to505,
                    '506':self.to506,'507':self.to507,'508':self.to508,'509':self.to509,'510':self.to510,
                    '511':self.to511,'512':self.to512,'513':self.to513,'514':self.to514,'515':self.to515,
                    '516':self.to516,'517':self.to517,'518':self.to518,'519':self.to519,'520':self.to520,
                    '521':self.to521,'522':self.to522,
                    '601':self.to601,'602':self.to602,'603':self.to603,'604':self.to604,'605':self.to605,
                    '606':self.to606,'607':self.to607,'608':self.to608,'609':self.to609,'610':self.to610,
                    '611':self.to611,'612':self.to612,'613':self.to613,'614':self.to614,'615':self.to615,
                    '616':self.to616,'617':self.to617,'618':self.to618,'619':self.to619,'620':self.to620,
                    '621':self.to621,'622':self.to622,
                    '701':self.to701,'702':self.to702,'703':self.to703,'704':self.to704,'705':self.to705,
                    '706':self.to706,'707':self.to707,'708':self.to708,'709':self.to709,'710':self.to710,
                    '711':self.to711,'712':self.to712,'713':self.to713,'714':self.to714,'715':self.to715,
                    '716':self.to716,'717':self.to717,'718':self.to718,'719':self.to719,'720':self.to720,
                    '721':self.to721,'722':self.to722,
                    '801':self.to801,'802':self.to802,'803':self.to803,'804':self.to804,'805':self.to805,
                    '806':self.to806,'807':self.to807,'808':self.to808,'809':self.to809,'810':self.to810,
                    '811':self.to811,'812':self.to812,'813':self.to813,'814':self.to814,'815':self.to815,
                    '816':self.to816,'817':self.to817,'818':self.to818,'819':self.to819,'820':self.to820,
                    '821':self.to821,'822':self.to822}

        rosGoal, wideYawTolerance = goalDic[goal]()
        return rosGoal, wideYawTolerance

    def getInitPose(self,floor):
        poseDic = {1:self.EVin1,2:self.EVin2,3:self.EVin3,4:self.EVin4,5:self.EVin5,6:self.EVin6,7:self.EVin7,8:self.EVin8}
        pose = poseDic[floor]()
        return pose

    def getELlist(self):
        ELlist = ['EVW1','EVW2','EVW3','EVW4','EVW5','EVW6','EVW7','EVW8']
        return ELlist

    def toLobby(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 2.11324628994
        goal.pose.position.y = 6.27
        goal.pose.orientation.z = 0
        goal.pose.orientation.w = 1
        wideYawTolerance = False
        return goal, wideYawTolerance

    # to101
    def to101(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 2.94501321267
        goal.pose.position.y = -3.77571291054
        goal.pose.orientation.z = 0
        goal.pose.orientation.w = 1
        wideYawTolerance = True
        return goal, wideYawTolerance

    # to102
    def to102(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 3.45246332406
        goal.pose.position.y = 3.8764963005
        goal.pose.orientation.z = 0
        goal.pose.orientation.w = 1
        wideYawTolerance = True
        return goal, wideYawTolerance

    # Lobby Standby Position = Charging pose-after docking
    def to100(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 2.09997223825
        goal.pose.position.y = 5.97498130148
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = False
        return goal, wideYawTolerance

    # Charging pose-after docking
    def to199(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 2.09997223825
        goal.pose.position.y = 5.97498130148
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = False
        return goal, wideYawTolerance

    # Charging pose-before docking
    # TODO : Need to update position.
    def to198(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 2.11324628994
        goal.pose.position.y = 6.25600236113
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = False
        return goal, wideYawTolerance

    def toEVW1(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 0.493860905743
        goal.pose.position.y = 7.31040853099
        goal.pose.orientation.z = 1
        goal.pose.orientation.w = 0
        wideYawTolerance = False
        return goal, wideYawTolerance

    def toEVW2(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 0.493860905743
        goal.pose.position.y = 7.31040853099 
        goal.pose.orientation.z = -0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = False
        return goal, wideYawTolerance

    def toEVW3(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 0.493860905743
        goal.pose.position.y = 7.31040853099
        goal.pose.orientation.z = -0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = False
        return goal, wideYawTolerance

    def toEVW4(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 0.493860905743
        goal.pose.position.y = 7.31040853099
        goal.pose.orientation.z = 1
        goal.pose.orientation.w = 0
        wideYawTolerance = False
        return goal, wideYawTolerance

    def toEVW5(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 18.7110679672
        goal.pose.position.y = 15.8778358171
        goal.pose.orientation.z = -0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = False
        return goal, wideYawTolerance

    def toEVW6(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 18.7110679672
        goal.pose.position.y = 15.8778358171
        goal.pose.orientation.z = -0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = False
        return goal, wideYawTolerance

    def toEVW7(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 18.7110679672
        goal.pose.position.y = 15.8778358171
        goal.pose.orientation.z = -0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = False
        return goal, wideYawTolerance

    def toEVW8(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 18.7110679672
        goal.pose.position.y = 15.8778358171
        goal.pose.orientation.z = -0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = False
        return goal, wideYawTolerance

    def toEVW1S(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = -0.727724030368
        goal.pose.position.y = 3.32772930553
        goal.pose.orientation.z = 1
        goal.pose.orientation.w = 0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def toEVW2S(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 19.318432305
        goal.pose.position.y = 15.0196775001
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def toEVW3S(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 19.318432305
        goal.pose.position.y = 15.0196775001
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def toEVW4S(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = -0.727724030368
        goal.pose.position.y = 3.32772930553
        goal.pose.orientation.z = 1
        goal.pose.orientation.w = 0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def toEVW5S(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 19.318432305
        goal.pose.position.y = 15.0196775001
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def toEVW6S(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 19.318432305
        goal.pose.position.y = 15.0196775001
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def toEVW7S(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 19.318432305
        goal.pose.position.y = 15.0196775001
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def toEVW8S(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 19.318432305
        goal.pose.position.y = 15.0196775001
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to201(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.3171449879
        goal.pose.position.y = 15.3831440537
        goal.pose.orientation.z = 0.7 
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to202(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.2082601368
        goal.pose.position.y = 19.2542101077
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to203(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.2396337392
        goal.pose.position.y = 22.1643964026
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to204(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.2396337392
        goal.pose.position.y = 22.1643964026
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to205(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.2396337392
        goal.pose.position.y = 22.1643964026
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to206(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 17.9255871086
        goal.pose.position.y = 20.5811601316
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to207(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 21.0047407562
        goal.pose.position.y = 20.5193257049
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to208(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 21.024852404
        goal.pose.position.y = 9.96519876903
        goal.pose.orientation.z = 1.0
        goal.pose.orientation.w = 0.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to209(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 18.0578675979
        goal.pose.position.y = 10.0703862359
        goal.pose.orientation.z = 1.0
        goal.pose.orientation.w = 0.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to210(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.4567143886
        goal.pose.position.y = 8.51149079777
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to211(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.4567143886
        goal.pose.position.y = 8.51149079777
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to212(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.3262009731
        goal.pose.position.y = 10.496049798
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to213(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.2398540038
        goal.pose.position.y = 13.8267106447
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance


    def to301(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.3569344103
        goal.pose.position.y = 16.8159943769
        goal.pose.orientation.z = 0.7 
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to302(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.188418016 
        goal.pose.position.y = 19.2327338152 
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to303(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.3619290422
        goal.pose.position.y = 21.9787036009
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to304(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.3619290422
        goal.pose.position.y = 21.9787036009
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to305(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.3619290422
        goal.pose.position.y = 21.9787036009
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to306(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 17.9082532014
        goal.pose.position.y = 20.6494628804
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to307(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 21.2815433789
        goal.pose.position.y = 20.6012823065
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to308(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 23.7717424953
        goal.pose.position.y = 20.6436194654
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to309(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 26.3220984759
        goal.pose.position.y = 20.6294432062
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to310(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 25.2468581375
        goal.pose.position.y = 20.6590750925
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to311(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 27.2339342337
        goal.pose.position.y = 18.5274241957
        goal.pose.orientation.z = -0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to312(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 27.2411810265
        goal.pose.position.y = 13.1902308096
        goal.pose.orientation.z = -0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to313(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 27.1839040341
        goal.pose.position.y = 10.4694447218
        goal.pose.orientation.z = -0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to314(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 25.20
        goal.pose.position.y = 10.0
        goal.pose.orientation.z = 1.0
        goal.pose.orientation.w = 0.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to315(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 26.7424575752
        goal.pose.position.y = 9.91965972346
        goal.pose.orientation.z = 1.0
        goal.pose.orientation.w = 0.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to316(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 23.5346621932
        goal.pose.position.y = 10.0731270935
        goal.pose.orientation.z = 1.0
        goal.pose.orientation.w = 0.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to317(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 21.30
        goal.pose.position.y = 10.0
        goal.pose.orientation.z = 1.0
        goal.pose.orientation.w = 0.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to318(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 18.0
        goal.pose.position.y = 10.0
        goal.pose.orientation.z = 1.0
        goal.pose.orientation.w = 0.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to319(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.5296231402
        goal.pose.position.y = 8.40519070231
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to320(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.5296231402
        goal.pose.position.y = 8.40519070231
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to321(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.4550745442
        goal.pose.position.y = 10.4600407111
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to322(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.2903074681
        goal.pose.position.y = 13.4112696929
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to401(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.3569344103
        goal.pose.position.y = 16.8159943769
        goal.pose.orientation.z = 0.7 
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to402(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.188418016 
        goal.pose.position.y = 19.2327338152 
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to403(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.3619290422
        goal.pose.position.y = 21.9787036009
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to404(self):
        # SAME AS 198
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = -0.727724030368
        goal.pose.position.y = 3.32772930553
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to405(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.3619290422
        goal.pose.position.y = 21.9787036009
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to406(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 17.9082532014
        goal.pose.position.y = 20.6494628804
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to407(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 21.2815433789
        goal.pose.position.y = 20.6012823065
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to408(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 23.7717424953
        goal.pose.position.y = 20.6436194654
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to409(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 26.3220984759
        goal.pose.position.y = 20.6294432062
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to410(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 25.2468581375
        goal.pose.position.y = 20.6590750925
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to411(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 27.2339342337
        goal.pose.position.y = 18.5274241957
        goal.pose.orientation.z = -0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to412(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 27.2411810265
        goal.pose.position.y = 13.1902308096
        goal.pose.orientation.z = -0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to413(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 27.1839040341
        goal.pose.position.y = 10.4694447218
        goal.pose.orientation.z = -0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to414(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 25.20
        goal.pose.position.y = 10.0
        goal.pose.orientation.z = 1.0
        goal.pose.orientation.w = 0.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to415(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 26.7424575752
        goal.pose.position.y = 9.91965972346
        goal.pose.orientation.z = 1.0
        goal.pose.orientation.w = 0.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to416(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 23.5346621932
        goal.pose.position.y = 10.0731270935
        goal.pose.orientation.z = 1.0
        goal.pose.orientation.w = 0.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to417(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 21.30
        goal.pose.position.y = 10.0
        goal.pose.orientation.z = 1.0
        goal.pose.orientation.w = 0.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to418(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 18.0
        goal.pose.position.y = 10.0
        goal.pose.orientation.z = 1.0
        goal.pose.orientation.w = 0.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to419(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.5296231402
        goal.pose.position.y = 8.40519070231
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to420(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.5296231402
        goal.pose.position.y = 8.40519070231
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to421(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.4550745442
        goal.pose.position.y = 10.4600407111
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to422(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.2903074681
        goal.pose.position.y = 13.4112696929
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to501(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.3569344103
        goal.pose.position.y = 16.8159943769
        goal.pose.orientation.z = 0.7 
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to502(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.188418016 
        goal.pose.position.y = 19.2327338152 
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to503(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.3619290422
        goal.pose.position.y = 21.9787036009
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to504(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.3619290422
        goal.pose.position.y = 21.9787036009
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to505(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.3619290422
        goal.pose.position.y = 21.9787036009
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to506(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 17.9082532014
        goal.pose.position.y = 20.6494628804
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to507(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 21.2815433789
        goal.pose.position.y = 20.6012823065
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to508(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 23.7717424953
        goal.pose.position.y = 20.6436194654
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to509(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 26.3220984759
        goal.pose.position.y = 20.6294432062
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to510(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 25.2468581375
        goal.pose.position.y = 20.6590750925
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to511(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 27.2339342337
        goal.pose.position.y = 18.5274241957
        goal.pose.orientation.z = -0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to512(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 27.2411810265
        goal.pose.position.y = 13.1902308096
        goal.pose.orientation.z = -0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to513(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 27.1839040341
        goal.pose.position.y = 10.4694447218
        goal.pose.orientation.z = -0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to514(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 25.20
        goal.pose.position.y = 10.0
        goal.pose.orientation.z = 1.0
        goal.pose.orientation.w = 0.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to515(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 26.7424575752
        goal.pose.position.y = 9.91965972346
        goal.pose.orientation.z = 1.0
        goal.pose.orientation.w = 0.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to516(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 23.5346621932
        goal.pose.position.y = 10.0731270935
        goal.pose.orientation.z = 1.0
        goal.pose.orientation.w = 0.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to517(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 21.30
        goal.pose.position.y = 10.0
        goal.pose.orientation.z = 1.0
        goal.pose.orientation.w = 0.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to518(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 18.0
        goal.pose.position.y = 10.0
        goal.pose.orientation.z = 1.0
        goal.pose.orientation.w = 0.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to519(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.5296231402
        goal.pose.position.y = 8.40519070231
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to520(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.5296231402
        goal.pose.position.y = 8.40519070231
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to521(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.4550745442
        goal.pose.position.y = 10.4600407111
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to522(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.2903074681
        goal.pose.position.y = 13.4112696929
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to601(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.3569344103
        goal.pose.position.y = 16.8159943769
        goal.pose.orientation.z = 0.7 
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to602(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.188418016 
        goal.pose.position.y = 19.2327338152 
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to603(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.3619290422
        goal.pose.position.y = 21.9787036009
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to604(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.3619290422
        goal.pose.position.y = 21.9787036009
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to605(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.3619290422
        goal.pose.position.y = 21.9787036009
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to606(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 17.9082532014
        goal.pose.position.y = 20.6494628804
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to607(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 21.2815433789
        goal.pose.position.y = 20.6012823065
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to608(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 23.7717424953
        goal.pose.position.y = 20.6436194654
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to609(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 26.3220984759
        goal.pose.position.y = 20.6294432062
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to610(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 25.2468581375
        goal.pose.position.y = 20.6590750925
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to611(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 27.2339342337
        goal.pose.position.y = 18.5274241957
        goal.pose.orientation.z = -0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to612(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 27.2411810265
        goal.pose.position.y = 13.1902308096
        goal.pose.orientation.z = -0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to613(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 27.1839040341
        goal.pose.position.y = 10.4694447218
        goal.pose.orientation.z = -0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to614(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 25.20
        goal.pose.position.y = 10.0
        goal.pose.orientation.z = 1.0
        goal.pose.orientation.w = 0.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to615(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 26.7424575752
        goal.pose.position.y = 9.91965972346
        goal.pose.orientation.z = 1.0
        goal.pose.orientation.w = 0.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to616(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 23.5346621932
        goal.pose.position.y = 10.0731270935
        goal.pose.orientation.z = 1.0
        goal.pose.orientation.w = 0.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to617(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 21.30
        goal.pose.position.y = 10.0
        goal.pose.orientation.z = 1.0
        goal.pose.orientation.w = 0.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to618(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 18.0
        goal.pose.position.y = 10.0
        goal.pose.orientation.z = 1.0
        goal.pose.orientation.w = 0.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to619(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.5296231402
        goal.pose.position.y = 8.40519070231
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to620(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.5296231402
        goal.pose.position.y = 8.40519070231
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to621(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.4550745442
        goal.pose.position.y = 10.4600407111
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to622(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.2903074681
        goal.pose.position.y = 13.4112696929
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to701(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.3569344103
        goal.pose.position.y = 16.8159943769
        goal.pose.orientation.z = 0.7 
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to702(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.188418016 
        goal.pose.position.y = 19.2327338152 
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to703(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.3619290422
        goal.pose.position.y = 21.9787036009
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to704(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.3619290422
        goal.pose.position.y = 21.9787036009
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to705(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.3619290422
        goal.pose.position.y = 21.9787036009
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to706(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 17.9082532014
        goal.pose.position.y = 20.6494628804
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to707(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 21.2815433789
        goal.pose.position.y = 20.6012823065
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to708(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 23.7717424953
        goal.pose.position.y = 20.6436194654
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to709(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 26.3220984759
        goal.pose.position.y = 20.6294432062
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to710(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 25.2468581375
        goal.pose.position.y = 20.6590750925
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to711(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 27.2339342337
        goal.pose.position.y = 18.5274241957
        goal.pose.orientation.z = -0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to712(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 27.2411810265
        goal.pose.position.y = 13.1902308096
        goal.pose.orientation.z = -0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to713(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 27.1839040341
        goal.pose.position.y = 10.4694447218
        goal.pose.orientation.z = -0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to714(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 25.20
        goal.pose.position.y = 10.0
        goal.pose.orientation.z = 1.0
        goal.pose.orientation.w = 0.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to715(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 26.7424575752
        goal.pose.position.y = 9.91965972346
        goal.pose.orientation.z = 1.0
        goal.pose.orientation.w = 0.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to716(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 23.5346621932
        goal.pose.position.y = 10.0731270935
        goal.pose.orientation.z = 1.0
        goal.pose.orientation.w = 0.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to717(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 21.30
        goal.pose.position.y = 10.0
        goal.pose.orientation.z = 1.0
        goal.pose.orientation.w = 0.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to718(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 18.0
        goal.pose.position.y = 10.0
        goal.pose.orientation.z = 1.0
        goal.pose.orientation.w = 0.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to719(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.5296231402
        goal.pose.position.y = 8.40519070231
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to720(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.5296231402
        goal.pose.position.y = 8.40519070231
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to721(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.4550745442
        goal.pose.position.y = 10.4600407111
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to722(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.2903074681
        goal.pose.position.y = 13.4112696929
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to801(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.3569344103
        goal.pose.position.y = 16.8159943769
        goal.pose.orientation.z = 0.7 
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to802(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.188418016 
        goal.pose.position.y = 19.2327338152 
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to803(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.3619290422
        goal.pose.position.y = 21.9787036009
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to804(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.3619290422
        goal.pose.position.y = 21.9787036009
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to805(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.3619290422
        goal.pose.position.y = 21.9787036009
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to806(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 17.9082532014
        goal.pose.position.y = 20.6494628804
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to807(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 21.2815433789
        goal.pose.position.y = 20.6012823065
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to808(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 23.7717424953
        goal.pose.position.y = 20.6436194654
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to809(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 26.3220984759
        goal.pose.position.y = 20.6294432062
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to810(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 25.2468581375
        goal.pose.position.y = 20.6590750925
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to811(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 27.2339342337
        goal.pose.position.y = 18.5274241957
        goal.pose.orientation.z = -0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to812(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 27.2411810265
        goal.pose.position.y = 13.1902308096
        goal.pose.orientation.z = -0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to813(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 27.1839040341
        goal.pose.position.y = 10.4694447218
        goal.pose.orientation.z = -0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to814(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 25.20
        goal.pose.position.y = 10.0
        goal.pose.orientation.z = 1.0
        goal.pose.orientation.w = 0.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to815(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 26.7424575752
        goal.pose.position.y = 9.91965972346
        goal.pose.orientation.z = 1.0
        goal.pose.orientation.w = 0.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to816(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 23.5346621932
        goal.pose.position.y = 10.0731270935
        goal.pose.orientation.z = 1.0
        goal.pose.orientation.w = 0.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to817(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 21.30
        goal.pose.position.y = 10.0
        goal.pose.orientation.z = 1.0
        goal.pose.orientation.w = 0.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to818(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 18.0
        goal.pose.position.y = 10.0
        goal.pose.orientation.z = 1.0
        goal.pose.orientation.w = 0.0
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to819(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.5296231402
        goal.pose.position.y = 8.40519070231
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to820(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.5296231402
        goal.pose.position.y = 8.40519070231
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to821(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.4550745442
        goal.pose.position.y = 10.4600407111
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance

    def to822(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 13.2903074681
        goal.pose.position.y = 13.4112696929
        goal.pose.orientation.z = 0.7
        goal.pose.orientation.w = 0.7
        wideYawTolerance = True
        return goal, wideYawTolerance


    def EVin1(self):
        initPose = PoseWithCovarianceStamped()
        initPose.header.frame_id = 'map'
        initPose.pose.pose.position.x = 2.47118949076
        initPose.pose.pose.position.y = 7.10287951836
        initPose.pose.pose.orientation.z = 1
        initPose.pose.pose.orientation.w = 0
        initPose.pose.covariance[0] = 0.25
        initPose.pose.covariance[7] = 0.25
        initPose.pose.covariance[35] = 0.066
        return initPose

    def EVin2(self):
        initPose = PoseWithCovarianceStamped()
        initPose.header.frame_id = 'map'
        initPose.pose.pose.position.x = 18.6976383125
        initPose.pose.pose.position.y = 18.2461644449
        initPose.pose.pose.orientation.z = -0.7
        initPose.pose.pose.orientation.w = 0.7
        initPose.pose.covariance[0] = 0.25
        initPose.pose.covariance[7] = 0.25
        initPose.pose.covariance[35] = 0.066
        return initPose

    def EVin3(self):
        initPose = PoseWithCovarianceStamped()
        initPose.header.frame_id = 'map'
        initPose.pose.pose.position.x = 18.6976383125
        initPose.pose.pose.position.y = 18.2461644449
        initPose.pose.pose.orientation.z = -0.7
        initPose.pose.pose.orientation.w = 0.7
        initPose.pose.covariance[0] = 0.25
        initPose.pose.covariance[7] = 0.25
        initPose.pose.covariance[35] = 0.066
        return initPose

    def EVin4(self):
        initPose = PoseWithCovarianceStamped()
        initPose.header.frame_id = 'map'
        initPose.pose.pose.position.x = 2.47118949076
        initPose.pose.pose.position.y = 7.10287951836
        initPose.pose.pose.orientation.z = 1
        initPose.pose.pose.orientation.w = 0
        initPose.pose.covariance[0] = 0.25
        initPose.pose.covariance[7] = 0.25
        initPose.pose.covariance[35] = 0.066
        return initPose

    def EVin5(self):
        initPose = PoseWithCovarianceStamped()
        initPose.header.frame_id = 'map'
        initPose.pose.pose.position.x = 18.6976383125
        initPose.pose.pose.position.y = 18.2461644449
        initPose.pose.pose.orientation.z = -0.7
        initPose.pose.pose.orientation.w = 0.7
        initPose.pose.covariance[0] = 0.25
        initPose.pose.covariance[7] = 0.25
        initPose.pose.covariance[35] = 0.066
        return initPose

    def EVin6(self):
        initPose = PoseWithCovarianceStamped()
        initPose.header.frame_id = 'map'
        initPose.pose.pose.position.x = 18.6976383125
        initPose.pose.pose.position.y = 18.2461644449
        initPose.pose.pose.orientation.z = -0.7
        initPose.pose.pose.orientation.w = 0.7
        initPose.pose.covariance[0] = 0.25
        initPose.pose.covariance[7] = 0.25
        initPose.pose.covariance[35] = 0.066
        return initPose

    def EVin7(self):
        initPose = PoseWithCovarianceStamped()
        initPose.header.frame_id = 'map'
        initPose.pose.pose.position.x = 18.6976383125
        initPose.pose.pose.position.y = 18.2461644449
        initPose.pose.pose.orientation.z = -0.7
        initPose.pose.pose.orientation.w = 0.7
        initPose.pose.covariance[0] = 0.25
        initPose.pose.covariance[7] = 0.25
        initPose.pose.covariance[35] = 0.066
        return initPose

    def EVin8(self):
        initPose = PoseWithCovarianceStamped()
        initPose.header.frame_id = 'map'
        initPose.pose.pose.position.x = 18.6976383125
        initPose.pose.pose.position.y = 18.2461644449
        initPose.pose.pose.orientation.z = -0.7
        initPose.pose.pose.orientation.w = 0.7
        initPose.pose.covariance[0] = 0.25
        initPose.pose.covariance[7] = 0.25
        initPose.pose.covariance[35] = 0.066
        return initPose

    def startPose(self):
        initPose = PoseWithCovarianceStamped()
        initPose.header.frame_id = 'map'
        initPose.pose.pose.position.x = 2.09997223825
        initPose.pose.pose.position.y = 5.97498130148
        initPose.pose.pose.orientation.z = 0.7
        initPose.pose.pose.orientation.w = 0.7
        initPose.pose.covariance[0] = 0.25
        initPose.pose.covariance[7] = 0.25
        initPose.pose.covariance[35] = 0.066
        return initPose
