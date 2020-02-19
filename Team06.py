

from captureAgents import CaptureAgent
import distanceCalculator
import random, time, util, sys
from game import Directions
import game
from util import nearestPoint

#################
# Team creation #
#################

"""
def createTeam(firstIndex, secondIndex, thirdIndex, isRed,
               first = 'OffensiveReflexAgent', second = 'DefensiveReflexAgent1',
               third = 'DefensiveReflexAgent2'):
"""

def createTeam(firstIndex, secondIndex, thirdIndex, isRed,
               first = 'OffensiveReflexAgent', second = 'DummyAgent',
               third = 'DummyAgent'):
  """
  This function should return a list of three agents that will form the
  team, initialized using firstIndex and secondIndex as their agent
  index numbers.  isRed is True if the red team is being created, and
  will be False if the blue team is being created.

  As a potentially helpful development aid, this function can take
  additional string-valued keyword arguments ("first" and "second" are
  such arguments in the case of this function), which will come from
  the --redOpts and --blueOpts command-line arguments to capture.py.
  For the nightly contest, however, your team will be created without
  any extra arguments, so you should make sure that the default
  behavior is what you want for the nightly contest.
  """
  return [eval(first)(firstIndex), eval(second)(secondIndex), eval(third)(thirdIndex)]

##########
# Agents #
##########

class ReflexCaptureAgent(CaptureAgent):
  """
  A base class for reflex agents that chooses score-maximizing actions
  """
 
  def registerInitialState(self, gameState):
    self.start = gameState.getAgentPosition(self.index)
    #print('mii')
    CaptureAgent.registerInitialState(self, gameState)

  def chooseAction(self, gameState):
    """
    Picks among the actions with the highest Q(s,a).
    """
    actions = gameState.getLegalActions(self.index)

    # You can profile your evaluation time by uncommenting these lines
    # start = time.time()
    values = [self.evaluate(gameState, a) for a in actions]
    # print 'eval time for agent %d: %.4f' % (self.index, time.time() - start)

    maxValue = max(values)
    bestActions = [a for a, v in zip(actions, values) if v == maxValue]
    foodLeft = len(self.getFood(gameState).asList())
    #print(foodLeft)

#    if foodLeft == 0:
#      bestDist = 9999
#      for action in actions:
#        successor = self.getSuccessor(gameState, action)
#        pos2 = successor.getAgentPosition(self.index)
#        dist = self.getMazeDistance(self.start,pos2)
#        if dist < bestDist:
#          bestAction = action
#          bestDist = dist
#      return bestAction

    return random.choice(bestActions)

  def getSuccessor(self, gameState, action):
    """
    Finds the next successor which is a grid position (location tuple).
    """
    successor = gameState.generateSuccessor(self.index, action)
    pos = successor.getAgentState(self.index).getPosition()
    if pos != nearestPoint(pos):
      # Only half a grid position was covered
      return successor.generateSuccessor(self.index, action)
    else:
      return successor

  def evaluate(self, gameState, action):
    """
    Computes a linear combination of features and feature weights
    """
    features = self.getFeatures(gameState, action)
    weights = self.getWeights(gameState, action)
    #print(features)
    #print('----------------')
    #print(features * weights)
    return features * weights

  def getFeatures(self, gameState, action):
    """
    Returns a counter of features for the state
    """
    features = util.Counter()
    successor = self.getSuccessor(gameState, action)
    features['successorScore'] = self.getScore(successor)
    return features

  def getWeights(self, gameState, action):
    """
    Normally, weights do not depend on the gamestate.  They can be either
    a counter or a dictionary.
    """
    return {'successorScore': 1.0}

#class OffensiveReflexAgent(ReflexCaptureAgent):
#  """
#  A reflex agent that seeks food. This is an agent
#  we give you to get an idea of what an offensive agent might look like,
#  but it is by no means the best or only way to build an offensive agent.
#  """
#  def getFeatures(self, gameState, action):
#    features = util.Counter()
#    successor = self.getSuccessor(gameState, action)
#    foodList = self.getFood(successor).asList()    
#    features['successorScore'] = -len(foodList)#self.getScore(successor)
#
#    # Compute distance to the nearest food
#    myPos = successor.getAgentState(self.index).getPosition()
#    if len(foodList) > 0: # This should always be True,  but better safe than sorry
#      #myPos = successor.getAgentState(self.index).getPosition()
#      minDistance = min([self.getMazeDistance(myPos, food) for food in foodList])
#      if minDistance<2:
#          features['distanceToFood'] = -20
#      else:
#          features['distanceToFood'] = minDistance
#      
#      enemies = [successor.getAgentState(i) for i in self.getOpponents(successor)]
#      target = [a for a in enemies if a.isPacman and a.getPosition() != None]
#      ghost = [a for a in enemies if a.isPacman==0 and a.getPosition() != None]
#      #if target insight
#      if len(target)>0:
#          dists = [self.getMazeDistance(myPos, a.getPosition()) for a in target]
#          #features['defensive']=min(dists)*100
#          if min(dists)==0:
#              features['defensive']=5000
#          elif min(dists)<2:
#              features['defensive']=100
#          else:
#              features['defensive']=0
#      else:
#          features['defensive']=0   
#    #if ghost insight
#      if len(ghost)>0:
#        dists = [self.getMazeDistance(myPos, a.getPosition()) for a in ghost]
#        for d in dists:
#            if d<3:
#                features['successorScore']=features['successorScore']-50
#      
#      return features
#      
#
#
#  def getWeights(self, gameState, action):
#    return {'successorScore': 100, 'distanceToFood': -1, 'defensive':20}
#    #return {'successorScore': 100, 'distanceToFood': -1, 'defensive':20, 'numInvaders': 1000, 'onDefense': 100, 'invaderDistance': 10, 'stop': -100, 'reverse': -5, 'numGhost':10, 'ghostDis':10, 'distance':100}
class OffensiveReflexAgent(ReflexCaptureAgent):
  """
  A reflex agent that seeks food. This is an agent
  we give you to get an idea of what an offensive agent might look like,
  but it is by no means the best or only way to build an offensive agent.
  """
  def getFeatures(self, gameState, action):
    features = util.Counter()
    successor = self.getSuccessor(gameState, action)
    foodList = self.getFood(successor).asList()    
    features['successorScore'] = -len(foodList)#self.getScore(successor)

#[0 2 4] || [1 3 5]
#    team=self.getTeam(gameState)
    #print(team[0])    
#    if team[0]==0:
#        #red
#        ghost=[1,3,5]
#    else:
#        #blue
#        ghost=[0,2,4]
#    
    # Compute distance to the nearest food
    myPos = successor.getAgentState(self.index).getPosition()
    if len(foodList) > 0: # This should always be True,  but better safe than sorry
      #myPos = successor.getAgentState(self.index).getPosition()
      minDistance = min([self.getMazeDistance(myPos, food) for food in foodList])
      features['distanceToFood'] = minDistance
    
          
#    for i in ghost:
#        pos=gameState.getAgentPosition(i);
#        d=10
#        if pos is not None:
#            d=abs(pos[0]-myPos[0])+abs(pos[1]-myPos[1])
#            #print(d)
#        if d<3:
#            features['successorScore']=features['successorScore']-30
#            
    enemies = [successor.getAgentState(i) for i in self.getOpponents(successor)]
    target = [a for a in enemies if a.isPacman and a.getPosition() != None]
    ghost = [a for a in enemies if a.isPacman==0 and a.getPosition() != None]
    #if target insight
    if len(target)>0:
        dists = [self.getMazeDistance(myPos, a.getPosition()) for a in target]
        features['defensive']=min(dists)
    #if ghost insight
    if len(ghost)>0:
        dists = [self.getMazeDistance(myPos, a.getPosition()) for a in ghost]
        for d in dists:
            if d<3:
                features['successorScore']=features['successorScore']-30
    #print(features['distanceToFood'])
    return features

  def getWeights(self, gameState, action):
    return {'successorScore': 100, 'distanceToFood': -1, 'defensive':1}

class DefensiveReflexAgent1(ReflexCaptureAgent):
  """
  A reflex agent that keeps its side Pacman-free. Again,
  this is to give you an idea of what a defensive agent
  could be like.  It is not the best or only way to make
  such an agent.
  """

  def getFeatures(self, gameState, action):
    features = util.Counter()
    successor = self.getSuccessor(gameState, action)

    myState = successor.getAgentState(self.index)
    myPos = myState.getPosition()

    # Computes whether we're on defense (1) or offense (0)
    features['onDefense'] = 1
    #if myState.isPacman: features['onDefense'] = 0
    if myState.isPacman: features['onDefense'] = -1
    

    # Computes distance to invaders we can see
    enemies = [successor.getAgentState(i) for i in self.getOpponents(successor)]
    invaders = [a for a in enemies if a.isPacman and a.getPosition() != None]
    features['numInvaders'] = len(invaders)
    #print(features['numInvaders'])
    if len(invaders) > 0:
      dists = [self.getMazeDistance(myPos, a.getPosition()) for a in invaders]
      features['invaderDistance'] = min(dists)

    if action == Directions.STOP: features['stop'] = 1
    rev = Directions.REVERSE[gameState.getAgentState(self.index).configuration.direction]
    if action == rev: features['reverse'] = 1
    
    #ghost insight
    ghosts = [a for a in enemies if a.isPacman==0 and a.getPosition() != None]
    features['numGhost']=len(ghosts)
    
    if len(ghosts)>0:
      dists = [self.getMazeDistance(myPos, a.getPosition()) for a in ghosts]
      features['ghostDis']=min(dists)
    
    #[0 2 4] || [1 3 5]
    team=self.getTeam(gameState)   
    if team[0]==0:
        #red
        d2=5
    else:
        #blue
        d2=4
    d2Pos=gameState.getAgentPosition(d2) 
    #d2State = gameState.getAgentState(d2)
    if d2Pos is not None:
        #14 | 15
        #d2Pos=gameState.getAgentPosition(d2)
        if d2Pos[0]==myPos[0]==14 or d2Pos[0]==myPos[0]==15:
            dis=abs(d2Pos[1]-myPos[1])+abs(d2Pos[0]-myPos[0])
            if dis < 3:
                features['distance']=50
            else:
                features['distance']=0
            #features['distance']=50
        else:
            features['distance']=0
    
    return features

  def getWeights(self, gameState, action):
    return {'numInvaders': -1000, 'onDefense': 100, 'invaderDistance': -10, 'stop': -100, 'reverse': -5, 'numGhost':-10, 'ghostDis':-10, 'distance':100}
    #return {'numInvaders': -1000, 'onDefense': 100, 'invaderDistance': -10, 'stop': -100, 'reverse': -2}


class DefensiveReflexAgent1(ReflexCaptureAgent):
  """
  A reflex agent that keeps its side Pacman-free. Again,
  this is to give you an idea of what a defensive agent
  could be like.  It is not the best or only way to make
  such an agent.
  """

  def getFeatures(self, gameState, action):
    features = util.Counter()
    successor = self.getSuccessor(gameState, action)

    myState = successor.getAgentState(self.index)
    myPos = myState.getPosition()

    # Computes whether we're on defense (1) or offense (0)
    features['onDefense'] = 1
    #if myState.isPacman: features['onDefense'] = 0
    if myState.isPacman: features['onDefense'] = -1
    

    # Computes distance to invaders we can see
    enemies = [successor.getAgentState(i) for i in self.getOpponents(successor)]
    invaders = [a for a in enemies if a.isPacman and a.getPosition() != None]
    features['numInvaders'] = len(invaders)
    #print(features['numInvaders'])
    if len(invaders) > 0:
      dists = [self.getMazeDistance(myPos, a.getPosition()) for a in invaders]
      features['invaderDistance'] = min(dists)

    if action == Directions.STOP: features['stop'] = 1
    rev = Directions.REVERSE[gameState.getAgentState(self.index).configuration.direction]
    if action == rev: features['reverse'] = 1
    
    #ghost insight
    ghosts = [a for a in enemies if a.isPacman==0 and a.getPosition() != None]
    features['numGhost']=len(ghosts)
    
    if len(ghosts)>0:
      dists = [self.getMazeDistance(myPos, a.getPosition()) for a in ghosts]
      features['ghostDis']=min(dists)
    
    #[0 2 4] || [1 3 5]
    team=self.getTeam(gameState)   
    if team[0]==0:
        #red
        d2=5
    else:
        #blue
        d2=4
    d2Pos=gameState.getAgentPosition(d2) 
    #d2State = gameState.getAgentState(d2)
    if d2Pos is not None:
        #14 | 15
        #d2Pos=gameState.getAgentPosition(d2)
        if d2Pos[0]==myPos[0]==14 or d2Pos[0]==myPos[0]==15:
            dis=abs(d2Pos[1]-myPos[1])+abs(d2Pos[0]-myPos[0])
            if dis < 3:
                features['distance']=-50
            else:
                features['distance']=0
            #features['distance']=50
        else:
            features['distance']=0
    
    return features

  def getWeights(self, gameState, action):
    return {'numInvaders': -1000, 'onDefense': 100, 'invaderDistance': -10, 'stop': -100, 'reverse': -5, 'numGhost':-10, 'ghostDis':-10, 'distance':100}
    #return {'numInvaders': -1000, 'onDefense': 100, 'invaderDistance': -10, 'stop': -100, 'reverse': -2}

class DefensiveReflexAgent2(ReflexCaptureAgent):
  """
  A reflex agent that keeps its side Pacman-free. Again,
  this is to give you an idea of what a defensive agent
  could be like.  It is not the best or only way to make
  such an agent.
  """

  def getFeatures(self, gameState, action):
    features = util.Counter()
    successor = self.getSuccessor(gameState, action)

    myState = successor.getAgentState(self.index)
    myPos = myState.getPosition()

    # Computes whether we're on defense (1) or offense (0)
    features['onDefense'] = 1
    #if myState.isPacman: features['onDefense'] = 0
    if myState.isPacman: features['onDefense'] = -1
    

    # Computes distance to invaders we can see
    enemies = [successor.getAgentState(i) for i in self.getOpponents(successor)]
    invaders = [a for a in enemies if a.isPacman and a.getPosition() != None]
    features['numInvaders'] = len(invaders)
    #print(features['numInvaders'])
    if len(invaders) > 0:
      dists = [self.getMazeDistance(myPos, a.getPosition()) for a in invaders]
      features['invaderDistance'] = min(dists)

    if action == Directions.STOP: features['stop'] = 1
    rev = Directions.REVERSE[gameState.getAgentState(self.index).configuration.direction]
    if action == rev: features['reverse'] = 1
    
    #ghost insight
    ghosts = [a for a in enemies if a.isPacman==0 and a.getPosition() != None]
    features['numGhost']=len(ghosts)
    
    if len(ghosts)>0:
      dists = [self.getMazeDistance(myPos, a.getPosition()) for a in ghosts]
      features['ghostDis']=min(dists)
    
    
    return features

  def getWeights(self, gameState, action):
    return {'numInvaders': -1000, 'onDefense': 100, 'invaderDistance': -10, 'stop': -100, 'reverse': -5, 'numGhost':-10, 'ghostDis':-10}
    #return {'numInvaders': -1000, 'onDefense': 100, 'invaderDistance': -10, 'stop': -100, 'reverse': -2}

#------------------------------------------------------------------------------
class DummyAgent(CaptureAgent):
  """
  A Dummy agent to serve as an example of the necessary agent structure.
  You should look at baselineTeam.py for more details about how to
  create an agent as this is the bare minimum.
  """

  def registerInitialState(self, gameState):
    """
    This method handles the initial setup of the
    agent to populate useful fields (such as what team
    we're on).

    A distanceCalculator instance caches the maze distances
    between each pair of positions, so your agents can use:
    self.distancer.getDistance(p1, p2)

    IMPORTANT: This method may run for at most 15 seconds.
    """

    '''
    Make sure you do not delete the following line. If you would like to
    use Manhattan distances instead of maze distances in order to save
    on initialization time, please take a look at
    CaptureAgent.registerInitialState in captureAgents.py.
    '''
    CaptureAgent.registerInitialState(self, gameState)

    '''
    Your initialization code goes here, if you need any.
    '''
    # declaration
    self.getWalls = gameState.getWalls()
    
    self.myPacmanIndex = []      # indices of my pacman
    self.opPacmanIndex = []      # indices of opponent pacman
    self.bestPos = []            # best position of pacman to stay at
    self.state = [-1, -1, -1]    # 0: aheading to bestPos
                                 # 1: guarding at bestPos
                                 # 2: eating pacman
                                 # 4: escape
    self.path = []
    self.path.append( [] )
    self.path.append( [] )
    self.path.append( [] )    
    
    # get indices of my pacman    
    self.myPacmanIndex = CaptureAgent.getTeam( self, gameState )
    self.opPacmanIndex = CaptureAgent.getOpponents( self, gameState )
 
    # get the best pos of my team
    if self.red:
        self.bestPos.append( (13, 6) )
        self.bestPos.append( (12, 13) )
        #self.bestPos.append( (13, 6) )
        self.bestPos.append( (11, 3) )
    else:
        self.bestPos.append( (18, 7) )
        self.bestPos.append( (20, 12) )
        #self.bestPos.append( (18, 7) )
        self.bestPos.append( (19, 2) )
    
    
  def chooseAction(self, gameState):
    self.getDisAll(gameState)    
    """
    Picks among actions randomly.
    """
    # which pacman is acting?    
    IndexOfActPacman = self.myPacmanIndex.index(self.index)
    
    #CaptureAgent.getFood(self,gameState)
    
    # pos of acting pacman
    pos = gameState.getAgentPosition(self.index)

    
    
    #Is about to ahead to bestPos
    if self.state[ IndexOfActPacman ] == -1:
        
        # find path to bestPos     
        self.path[IndexOfActPacman] = self.BFS( gameState, pos, self.bestPos[ IndexOfActPacman ] )
        self.state[ IndexOfActPacman ] = 0
    
    
    # ahead to bestPos
    if self.state[ IndexOfActPacman ] == 0:
        # I arrive
        if len( self.path[IndexOfActPacman] ) == 0:
            self.state[ IndexOfActPacman ] = 1
            
        # Interrupt
        
        
        # Ahead to bestPos  
        else:
            act = self.path[ IndexOfActPacman ][0]
            self.path[ IndexOfActPacman ].pop(0)
            return act

    
    # guarding    
    if self.state[ IndexOfActPacman ] == 1:
        act = self.HillClimbing( gameState, pos )
        
        if act == -1:   # no pacman exist: go back to best pos
            # find path to bestPos     
            self.path[IndexOfActPacman] = self.BFS( gameState, pos, self.bestPos[ IndexOfActPacman ] )
            self.state[ IndexOfActPacman ] = 0
            return 'Stop'
        else:
            return act
                
        


  def BFS( self, gameState, start, target ):
    path = []
    action = {}         # action(x, y) = ( dir it from, (orig_x, orig_y) )
    visited = []
    waitForSearch = []
    pos = start
    
    visited.append( start )
    
    while not( pos == target ): 
        neighbor = []
        neighbor.append( (pos[0] + 1, pos[1], 'East' ) )
        neighbor.append( (pos[0] - 1, pos[1], 'West' ) )
        neighbor.append( (pos[0], pos[1] + 1, 'North' ) )
        neighbor.append( (pos[0], pos[1] - 1, 'South' ) )
        
        for node in neighbor:
            nodePos = ( node[0], node[1] )
            if node[0] >= 0 and node[1] >= 0 and node[0] <= 30 and node[1] <= 22:
                if not( nodePos in visited ):
                    if not gameState.hasWall( node[0], node[1] ):
                        visited.append( nodePos )
                        waitForSearch.append( nodePos )
                        action[ nodePos ] = ( node[2], pos )
        
        pos = ( waitForSearch[0][0], waitForSearch[0][1] )
        waitForSearch.pop(0)
    
    node = target
    while not node == start:
        path.append( action[ node ][0] )
        node = action[ node ][1]
    path.reverse()     
    return path
    
  def HillClimbing( self, gameState, pos ): 
    # get neighbors
    neighbor = []
    act = []
    neighbor.append( (pos[0], pos[1], 'Stop' ) )
    neighbor.append( (pos[0] + 1, pos[1], 'East' ) )
    neighbor.append( (pos[0] - 1, pos[1], 'West' ) )
    neighbor.append( (pos[0], pos[1] + 1, 'North' ) )
    neighbor.append( (pos[0], pos[1] - 1, 'South' ) )
    
    # choose pos with highest score    
    Score = []
    minScore = -999
    index = 0
    
    for node in neighbor:
        if not gameState.hasWall( node[0], node[1] ):        
            if node[0] >= 0 and node[1] >= 0 and node[0] <= 30 and node[1] <= 22:
                Score.append( self.ScoreGhost( gameState, ( node[0], node[1] ) ) )
                act.append( node[2] )
    
    minScore = max( Score )
    index = Score.index( minScore )
    
    
    # if no pacman exist, go back to bestPos
    if minScore == -999:
        return -1        
    else:
        return act[index]
        
  
  # EvaluationFunction when being ghost  
  def ScoreGhost( self, gameState, pos ):   
    Score = -999
    
    # get opponent dis  
    opDis = []
    for i in range(3):
        tmp = gameState.getAgentPosition( self.opPacmanIndex[ i ] )
        if tmp:
            if CaptureAgent.getMazeDistance( self, pos, tmp ) < 6:
                opDis.append( CaptureAgent.getMazeDistance( self, pos, tmp ) )
    
    # choose the min of opDis
    if len( opDis ) > 0:
        Score = 0 - min( opDis )
    
    if self.red:
        if pos[0] > 15:
            Score = -9999
    else:
        if pos[0] <= 15:
            Score = -9999
    
    return Score  
      
     
  def getDisAll( self, gameState ):
    disAll = []
    tmp = []
      
    # get 7 times of getting 100 times of getAgentDistances
    #for i in range( 7 ):
        
    # get 100 times of getAgentDistances
    tmp = gameState.getAgentDistances()
    tmp2 = gameState.getAgentDistances()
   
    #print( tmp )
    return tmp    
            
            
              