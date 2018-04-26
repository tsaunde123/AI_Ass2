% candidate_number(12345).

% Find hidden identity by repeatedly calling agent_ask_oracle(oscar,o(1),link,L)
% find_identity(-A)
find_identity(A):-
  (part_module(1) -> solve_task(Task, Cost)
  ;part_module(2) -> find_identity_2(A)
  ;part_module(3) -> find_identity_o(A)
  ).

find_identity_2(A):-
  findall(Actor, actor(Actor), Actors), %get all actors
  find_identity_2(A, Actors).
find_identity_2(A, Actors) :-
  agent_ask_oracle(oscar,o(1),link,L),
  findall(Actor,(wp:actor_links(Actor,Links), member(L, Links), member(Actor, Actors)), FilteredActors),
  writeln("Possible:" : FilteredActors),
  (FilteredActors = [Answer|[]] -> A = Answer
  ;otherwise -> find_identity_2(A, FilteredActors)
  ).

find_identity_2(A):-
  findall(Actor, actor(Actor), Actors), %get all actors
  find_identity_2(A, Actors).
find_identity_2(A, Actors) :-
  agent_ask_oracle(oscar,o(1),link,L),
  findall(Actor,(wp:actor_links(Actor,Links), member(L, Links), member(Actor, Actors)), FilteredActors),
  writeln("Possible:" : FilteredActors),
  (FilteredActors = [Answer|[]] -> A = Answer
  ;otherwise -> find_identity_2(A, FilteredActors)
  ).

find_identity_o(A):-
  writeln('Running our function'),
  %A='Not yet implemented'.
  my_agent(Agent),
  query_world( agent_current_position, [Agent,P] ),
  findall(Actor, actor(Actor), Actors),
  writeln('Actors':Actors),
  VisitedOracles = [],
  writeln('Query world':P),
  solve_task_bfsP(find(c(1)),[[c(0,0,P),P]],0,_,_,_,CPos1,_),!,
  writeln('CPos1':CPos1),
  solve_task_bfsP(find(c(2)),[[c(0,0,P),P]],0,_,_,_,CPos2,_),!,
  writeln('CPos2':CPos2),
  append([CPos1],[],ChargePointsPos1), append([CPos2],ChargePointsPos1,ChargePointsPosList),
  writeln(ChargePointsPosList),
  pathToOracle(Agent,CPos1,DistToCP1,Path1), %path to CP1
  pathToOracle(Agent,CPos2,DistToCP2,Path2), %path to CP2
  (DistToCP1 < DistToCP2 -> goCP(Agent,CPos1),
                            navigateMap(Agent,VisitedOracles,CPos1,Actors,ActorGuess)
  ;otherwise             -> goCP(Agent,CPos2),
                            navigateMap(Agent,VisitedOracles,CPos2,Actors,ActorGuess)),
  A=ActorGuess.
  % closestCP(Agent,ChargePointsPosList,ClosestCP,DistToClosest),
  % goCP(Agent,ClosestCP),
  % navigateMap(Agent,VisitedOracles,ChargePointsPosList,Actors).


navigateMap(Agent,VisitedOracles,ChargePos,Actors,A) :-
  query_world( agent_current_position, [Agent,P] ),
  solve_task_bfsP(find(o(_)),[[c(0,0,P),P]],0,_,_,_,OPos,OID),%!, %find oracle Pos % do i need exclamation mark?
  writeln('Found nearest Oracle':OPos),
  writeln('OracleIDDDDDDDDDDDDDDDDDDDDDDDDDDDDD is':OID),
  \+member(OPos,VisitedOracles), %check oracle not visited yet
  writeln('Checked if its a member'),
  pathToOracle(Agent,OPos,DistToOracle,Path), %find exact path distance to oracle
  writeln('Got path to oracle':Path),
  query_world( agent_current_energy, [Agent,Energy] ),
  (Energy > ((DistToOracle*2)+11) -> writeln('Enough fuel to visit oracle'),
                                     query_world(agent_do_moves,[Agent,Path]), %if enough fuel to go there, ask question, and back
                                     writeln('Arrived at oracle'),
                                     query_world(agent_ask_oracle,[Agent,OID,link,L]),
                                     writeln('Link':L),
                                     goCP(Agent,ChargePos), %can just reverse path and to go back to oracle
                                     append([OPos],VisitedOracles,NewVisitedOracles)
  ;writeln('Not enough fuel to get there and back to charge station')
  ),
  findall(Actor,(wp:actor_links(Actor,Links), member(L, Links), member(Actor, Actors)), FilteredActors),
  writeln("POSSIBLE ACTORSSSSSSSS:" : FilteredActors),
  (FilteredActors = [Answer|[]] -> A = Answer
  ;otherwise -> writeln('Recursive Call'),
                navigateMap(Agent,NewVisitedOracles,ChargePos,FilteredActors,A)).


pathToOracle(Agent,OraclePos,DistToOracle,Path) :-
  writeln('In pathToOracle'),
  query_world( agent_current_position, [Agent,P] ),
  G0 is 0,
  map_distance(P,OraclePos,H0),
  writeln('Fails here?'),
  F0 is G0+H0,
  writeln('Finding adjacent nodes'),
  findall(t(Dist,Adj_pos), %find adjacent neighbor to CP
       (map_adjacent(OraclePos,Adj_pos,empty),map_distance(P,Adj_pos,Dist)),
        Neighbors),
  sort(Neighbors,SNeighbors),
  writeln('Found adjacent nodes'),
  SNeighbors = [t(D,N)|_],
  writeln('N':N),
  solve_task_AstarP(go(N),[[c(F0,G0,P),P]],0,R,Cost,_NewPos),
  reverse(R,[_Init|Path]),
  length(Path,DistToOracle).

goCP(Agent,CPPos) :-
  query_world( agent_current_position, [Agent,P] ),
  G0 is 0, % else use Astar search
  map_distance(P,CPPos,H0), % original estimate of cost to get to goal
  F0 is G0+H0,
  findall(t(Dist,Adj_pos),
       (map_adjacent(CPPos,Adj_pos,empty),map_distance(P,Adj_pos,Dist)),
        Neighbors),
  sort(Neighbors,SNeighbors),
  writeln('SNeighbors':SNeighbors),
  SNeighbors = [t(D,N)|_],
  %map_adjacent(CPPos,N,empty),
  writeln('Adj pos':N),
  solve_task_AstarP(go(N),[[c(F0,G0,P),P]],0,R,Cost,_NewPos),%!,
  writeln('RPath':R),
  reverse(R,[_Init|Path]),
  writeln('Path':Path),
  query_world( agent_current_position, [Agent,P2] ),
  writeln(P2),
  query_world( agent_do_moves, [Agent,Path] ),
  writeln('Agent Moved!!!'),
  query_world( agent_current_energy, [Agent,EnergyBefore] ),
  writeln('Energy before topup':EnergyBefore),
  query_world(agent_topup_energy, [Agent,c(_)]),
  query_world( agent_current_energy, [Agent,EnergyAfter] ),
  writeln('Energy after topup':EnergyAfter).

closestCP(Agent,ChargePoints,Closest,DistToClosest) :-
  writeln('In closestCP'),
  writeln('ChargePoints':ChargePoints),
  ChargePoints=[CPos1,CPos2],
  writeln('ChargePoints decomp works'),
  pathToOracle(Agent,CPos1,DistToCP1,Path1), %path to CP1
  pathToOracle(Agent,CPos2,DistToCP2,Path2), %path to CP2
  writeln('DistToCP1':DistToCP1),
  writeln('DistToCP2':DistToCP2),
  (DistToCP1 < DistToCP2 -> writeln('DistToCP1 < DistToCP2'),
                            Closest = CPos1,
                            writeln('Closest':Closest),
                            DistToClosest is DistToCP1
  ;otherwise -> Closest = CPos2,
                            DistToClosest is DistToCP2).


% Breadth-first search for o(Goal) and c(Goal) as don't know location of oracles and charging points
solve_task_bfsP(Task,Agenda,Depth,RPath,[cost(Cost),depth(Depth)],NewPos,OPos,O) :-
  % writeln('Task':Task),
  getClosestOraclePos(Task,Agenda,RPath,Cost,NewPos,OPos,O).
  % writeln('HERE').
solve_task_bfsP(Task,Agenda,D,RR,Cost,NewPos,OPos,O) :-
  % writeln('call to main bfs'),
  Agenda = [[c(F,F,P)|RPath]|Paths], % dont know position of o and c bfs -> no heuristic/ estimated cost to goal
  %Task = find(Goal),
  findall([c(F1,F1,P1),R|RPath],
    (search(P,P1,R,C),
     \+memberchk(R,RPath),
     F1 is F+C,
     \+member([c(_,_,P1)|_],Paths)),
    Children),
  %sort(Children,SChildren),
  D1 is D+1,
  append(Paths, Children, NewAgenda), % not sure why we just append...
  % writeln('call to end of main bfs'),
  solve_task_bfsP(Task,NewAgenda,D1,RR,Cost,NewPos,OPos,O).

% A-star search for go(Goal)
solve_task_AstarP(Task,Agenda,Depth,RPath,[cost(Cost),depth(Depth)],NewPos) :-
  achievedP(Task,Agenda,RPath,Cost,NewPos).
solve_task_AstarP(Task,Agenda,D,RR,Cost,NewPos) :-
  Agenda = [[c(F,G,P)|RPath]|Paths],
  % writeln(Agenda),
  Task = go(Goal),
  findall([c(F1,G1,P1),R|RPath], %first arg of findall is template of return value (what the return value should look like)
    (search(P,P1,R,C), % find all children of curr pos
     \+ memberchk(R,RPath), % check that returned child isnt already in path
     G1 is G+C, % G is cost to arrive to curr pos, update G with cost to move to child
     map_distance(P1,Goal,H), % calc H that is heuristic (estimated cost) to get to goal
     F1 is G1+H,
     \+memberchk([c(_,_,P1)|_],Paths)), % update F which is total estimated cost to get to goal
    Children), % store result in Children
  sort(Children,SChildren),
  % writeln(SChildren),
  D1 is D+1,
  append(Paths,SChildren,NewAgenda), sort(NewAgenda, SNewAgenda), % append and sort is equivalent to merge
  %mergelists(SChildren,Paths,SNewAgenda), % doesnt work... so using append+sort for now
  solve_task_AstarP(Task,SNewAgenda,D1,RR,Cost,NewPos).

getClosestOraclePos(find(O),Current,RPath,Cost,NewPos,OPos,O) :-
  % writeln(O),
  % writeln('Called Base Case'),
  Current = [[c(Cost,_,NewPos)|RPath]|_],
  % writeln('Current'),
  ( O=none    -> true
  ; otherwise -> RPath = [Last|_],map_adjacent(Last,OPos,O)%,checkOracleVisited(O)
  ).
  % writeln('OPos':OPos).

achievedP(go(Exit),Current,RPath,Cost,NewPos) :-
  Current = [[c(Cost,_,NewPos)|RPath]|_],
  ( Exit=none -> true
  ; otherwise -> RPath = [Exit|_]
  ).

checkOracleVisited(Oracle):-
  my_agent(Agent),
  (query_world(agent_check_oracle, [Agent, o(X)]) -> false
  ;otherwise -> true).



%Call bfs to find all charging stations/ oracles
%Then call Astar to see if you can access it, if not call Astar on the next closest one
%Add visited oracles to list so you don't revisit them










%always keep track of of closest charging station
%keep looking for oracles until you are at a level of fuel = distanceToClosestChargeStation + 1 + 10 (price of oracle query)
