% candidate_number(12345).

% Find hidden identity by repeatedly calling agent_ask_oracle(oscar,o(1),link,L)
% find_identity(-A)
find_identity(A):-
  writeln('ran find_identity(A)'),
  (part_module(2)   -> find_identity_2(A)
  ; otherwise -> find_identity_o(A)
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
  writeln('Agent'),
  query_world( agent_current_position, [Agent,P] ),
  writeln('Query world'),
  findall(OPos, %find and save all CP locations
    (solve_task_bfs(find(c(X)),[[c(0,0,P),P]],0,R,Cost,_NewPos,OPos)),
    ChargePointsPos),
  writeln('FindAll'),
  writeln('ChargePointsPos':ChargePointsPos),
  getListHead(ChargePointsPos,firstChargePoint), %go to first CP as safety measure
  G0 is 0, % else use Astar search
  map_distance(P,firstChargePoint,H0), % original estimate of cost to get to goal
  F0 is G0+H0,
  %solve_task_Astar(Task,[[c(F0,G0,P),P]],0,R,Cost,_NewPos)
  solve_task_Astar(go(firstChargePoint),[[c(F0,G0,P),P]],0,R,Cost,_NewPos).

getListHead([]).
getListHead([Head|[]], Head).
getListHead([Head|Tail], Head).


%
% getClosestOraclePos(find(O),Current,RPath,Cost,NewPos,OPos) :-
%   Current = [[c(Cost,_,NewPos)|RPath]|_],
%   ( O=none    -> true
%   ; otherwise -> RPath = [Last|_],map_adjacent(Last,OPos,O)
%   ).
%
% VisitedOracles = [],
% my_agent(Agent),
% query_world( agent_current_position, [Agent,P] ),
% (Goal = o(X) -> solve_task_bfs(Task,[[c(0,0,P),P]],0,R,Cost,_NewPos,OPos) % if we need to find oracle or charge point
% ;Goal = c(X) -> solve_task_bfs(Task,[[c(0,0,P),P]],0,R,Cost,_NewPos,OPos) % use bfs as goal pos is unknown
% ;otherwise -> G0 is 0, % else use Astar search
%               map_distance(P,Goal,H0), % original estimate of cost to get to goal
%               F0 is G0+H0,
%               solve_task_Astar(Task,[[c(F0,G0,P),P]],0,R,Cost,_NewPos)
%
% findOracle() :-
%   solve_task_bfs(Task,[[c(0,0,P),P]],0,R,Cost,_NewPos,OPos), %return pos of nearest charging point
%   (member(OPos,VisitedOracles) -> findOracle()
%   ;)
%   agent_current_energy(Agent, Energy),
%   solve_task_Astar(Task,[[c(F0,G0,P),P]],0,R,Cost,_NewPos)
%   G0 is 0, % use Astar search to go to pos
%   map_distance(P,OPos,H0), % original estimate of cost to get to goal
%   (Energy > (H0+10) -> F0 is G0+H0,
%                        solve_task_Astar(Task,[[c(F0,G0,P),P]],0,R,Cost,_NewPos)
%   ;otherwise -> findChargingPoint()
%   )
%
%   )
%   F0 is G0+H0,
%   solve_task_Astar(Task,[[c(F0,G0,P),P]],0,R,Cost,_NewPos).
%
% findChargingPoint() :-
%   solve_task_bfs(Task,[[c(0,0,P),P]],0,R,Cost,_NewPos,OPos),
%   G0 is 0, % else use Astar search
%   map_distance(P,Goal,H0), % original estimate of cost to get to goal
%   F0 is G0+H0,
%   solve_task_Astar(Task,[[c(F0,G0,P),P]],0,R,Cost,_NewPos).
%
% append(oracle,VisitedOracles, NewVisitedOracles),
%
% Breadth-first search for o(Goal) and c(Goal) as don't know location of oracles and charging points
solve_task_bfs(Task,Agenda,Depth,RPath,[cost(Cost),depth(Depth)],NewPos,OPos) :-
  getClosestOraclePos(find(O),Current,RPath,Cost,NewPos,OPos).
solve_task_bfs(Task,Agenda,D,RR,Cost,NewPos,OPos) :-
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
  solve_task_bfs(Task,NewAgenda,D1,RR,Cost,NewPos,OPos).

% A-star search for go(Goal)
solve_task_Astar(Task,Agenda,Depth,RPath,[cost(Cost),depth(Depth)],NewPos) :-
  achieved(Task,Agenda,RPath,Cost,NewPos).
solve_task_Astar(Task,Agenda,D,RR,Cost,NewPos) :-
  Agenda = [[c(F,G,P)|RPath]|Paths],
  writeln(Agenda),
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
  writeln(SChildren),
  D1 is D+1,
  append(Paths,SChildren,NewAgenda), sort(NewAgenda, SNewAgenda), % append and sort is equivalent to merge
  %mergelists(SChildren,Paths,SNewAgenda), % doesnt work... so using append+sort for now
  solve_task_Astar(Task,SNewAgenda,D1,RR,Cost,NewPos).

getClosestOraclePos(find(O),Current,RPath,Cost,NewPos,OPos) :-
  writeln('Called Base Case'),
  Current = [[c(Cost,_,NewPos)|RPath]|_],
  ( O=none    -> true
  ; otherwise -> RPath = [Last|_],map_adjacent(Last,OPos,O)
  ),
  writeln('OPos':OPos)
.

achieved(go(Exit),Current,RPath,Cost,NewPos) :-
  Current = [[c(Cost,_,NewPos)|RPath]|_],
  ( Exit=none -> true
  ; otherwise -> RPath = [Exit|_]
  ).

%Call bfs to find all charging stations/ oracles
%Then call Astar to see if you can access it, if not call Astar on the next closest one
%Add visited oracles to list so you don't revisit them

%always keep track of of closest charging station
%keep looking for oracles until you are at a level of fuel = distanceToClosestChargeStation + 1 + 10 (price of oracle query)
