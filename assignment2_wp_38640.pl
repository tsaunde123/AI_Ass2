candidate_number(38640).

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
  my_agent(Agent),
  query_world( agent_current_position, [Agent,P] ),
  findall(Actor, actor(Actor), Actors),
  writeln('Actors':Actors),
  VisitedOracles = [],
  solve_task_bfsP(find(c(1)),[[c(0,0,P),P]],0,_,_,_,CPos1,_),!,
  writeln('Charging Point 1 Position':CPos1),
  solve_task_bfsP(find(c(2)),[[c(0,0,P),P]],0,_,_,_,CPos2,_),!,
  writeln('Charging Point 2 Position':CPos2),
  closestCP(Agent,CPos1,CPos2,ClosestCP,PathToClosest),
  goCP(Agent,PathToClosest),
  navigateMap(Agent,VisitedOracles,CPos1,CPos2,Actors,ActorGuess),
  A=ActorGuess.

navigateMap(Agent,VisitedOracles,CPos1,CPos2,Actors,A) :-
  query_world( agent_current_position, [Agent,P] ),
  solve_task_bfsP(find(o(_)),[[c(0,0,P),P]],0,_,_,_,OPos,OID),%!, %find oracle Pos
  \+member(OPos,VisitedOracles), %check oracle not visited yet
  pathToOracle(Agent,OPos,DistToOracle,Path), %find exact path distance to oracle
  query_world( agent_current_energy, [Agent,Energy] ),
  (Energy > ((DistToOracle*2)+11) -> query_world(agent_do_moves,[Agent,Path]), %if enough fuel to go there, ask question, and back
                                     query_world(agent_ask_oracle,[Agent,OID,link,L]),
                                     writeln('Link':L),
                                     append([OPos],VisitedOracles,NewVisitedOracles),
                                     writeln('Visited Oracles':NewVisitedOracles),
                                     closestCP(Agent,CPos1,CPos2,ClosestCP,PathToClosest),
                                     goCP(Agent,PathToClosest)
  ;writeln('Not enough fuel to get there and back to charge station')
  ),
  findall(Actor,(wp:actor_links(Actor,Links), member(L, Links), member(Actor, Actors)), FilteredActors),
  writeln("Possible Actors" : FilteredActors),
  (FilteredActors = [Answer|[]] -> A = Answer
  ;otherwise -> navigateMap(Agent,NewVisitedOracles,CPos1,CPos2,FilteredActors,A)).


pathToOracle(Agent,OraclePos,DistToOracle,Path) :-
  query_world( agent_current_position, [Agent,P] ),
  G0 is 0,
  map_distance(P,OraclePos,H0),
  F0 is G0+H0,
  findNearestNeighbor(P,OraclePos,N),
  solve_task_AstarP(go(N),[[c(F0,G0,P),P]],0,R,Cost,_NewPos),
  reverse(R,[_Init|Path]),
  length(Path,DistToOracle).

goCP(Agent,CPPath) :-
  query_world( agent_do_moves, [Agent,CPPath] ),
  query_world(agent_topup_energy, [Agent,c(_)]).


closestCP(Agent,CPos1,CPos2,Closest,PathToClosest) :-
  pathToOracle(Agent,CPos1,DistToCP1,Path1), %path to CP1
  pathToOracle(Agent,CPos2,DistToCP2,Path2), %path to CP2
  (DistToCP1 < DistToCP2 -> Closest = CPos1,
                            PathToClosest = Path1
  ;otherwise             -> Closest = CPos2,
                            PathToClosest = Path2).

findNearestNeighbor(CurrPos,Pos,N) :-
  findall(t(Dist,Adj_pos),
       (map_adjacent(Pos,Adj_pos,empty),map_distance(CurrPos,Adj_pos,Dist),map_adjacent(Adj_pos,R,empty)),
        Neighbors),
  sort(Neighbors,SNeighbors),
  SNeighbors = [t(D,N)|_].

% Breadth-first search for o(Goal) and c(Goal) as don't know location of oracles and charging points
solve_task_bfsP(Task,Agenda,Depth,RPath,[cost(Cost),depth(Depth)],NewPos,OPos,O) :-
  getClosestOraclePos(Task,Agenda,RPath,Cost,NewPos,OPos,O).
solve_task_bfsP(Task,Agenda,D,RR,Cost,NewPos,OPos,O) :-
  Agenda = [[c(F,F,P)|RPath]|Paths], % dont know position of o and c bfs -> no heuristic/ estimated cost to goal
  findall([c(F1,F1,P1),R|RPath],
    (search(P,P1,R,C),
     \+memberchk(R,RPath),
     F1 is F+C,
     \+member([c(_,_,P1)|_],Paths)),
    Children),
  D1 is D+1,
  append(Paths, Children, NewAgenda),
  solve_task_bfsP(Task,NewAgenda,D1,RR,Cost,NewPos,OPos,O).

% A-star search for go(Goal)
solve_task_AstarP(Task,Agenda,Depth,RPath,[cost(Cost),depth(Depth)],NewPos) :-
  achievedP(Task,Agenda,RPath,Cost,NewPos).
solve_task_AstarP(Task,Agenda,D,RR,Cost,NewPos) :-
  Agenda = [[c(F,G,P)|RPath]|Paths],
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
  D1 is D+1,
  append(Paths,SChildren,NewAgenda), sort(NewAgenda, SNewAgenda), % append and sort is equivalent to merge
  solve_task_AstarP(Task,SNewAgenda,D1,RR,Cost,NewPos).

getClosestOraclePos(find(O),Current,RPath,Cost,NewPos,OPos,O) :-
  Current = [[c(Cost,_,NewPos)|RPath]|_],
  ( O=none    -> true
  ; otherwise -> RPath = [Last|_],map_adjacent(Last,OPos,O)
  ).

achievedP(go(Exit),Current,RPath,Cost,NewPos) :-
  Current = [[c(Cost,_,NewPos)|RPath]|_],
  ( Exit=none -> true
  ; otherwise -> RPath = [Exit|_]
  ).
