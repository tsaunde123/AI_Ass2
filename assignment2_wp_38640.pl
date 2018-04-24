% candidate_number(12345).

% Find hidden identity by repeatedly calling agent_ask_oracle(oscar,o(1),link,L)
% find_identity(-A)
find_identity(A):-
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
  A='Not yet implemented'.
  my_agent(Agent),
  query_world( agent_current_position, [Agent,P] ),
  Task =.. [Command,Goal],
  (Goal = o(X) -> solve_task_bfs(Task,[[c(0,0,P),P]],0,R,Cost,_NewPos) % if we need to find oracle or charge point
  ;Goal = c(X) -> solve_task_bfs(Task,[[c(0,0,P),P]],0,R,Cost,_NewPos) % use bfs as goal pos is unknown
  ;otherwise -> G0 is 0, % else use Astar search
                map_distance(P,Goal,H0), % original estimate of cost to get to goal
                F0 is G0+H0,
                solve_task_Astar(Task,[[c(F0,G0,P),P]],0,R,Cost,_NewPos)
  ),
  !,
  reverse(R,[_Init|Path]),
  query_world( agent_do_moves, [Agent,Path] ).

getClosestOraclePos(find(O),Current,RPath,Cost,NewPos,OPos) :-
  Current = [[c(Cost,_,NewPos)|RPath]|_],
  ( O=none    -> true
  ; otherwise -> RPath = [Last|_],map_adjacent(Last,OPos,O)
  ).

VisitedOracles = [],
my_agent(Agent),
query_world( agent_current_position, [Agent,P] ),
(Goal = o(X) -> solve_task_bfs(Task,[[c(0,0,P),P]],0,R,Cost,_NewPos,OPos) % if we need to find oracle or charge point
;Goal = c(X) -> solve_task_bfs(Task,[[c(0,0,P),P]],0,R,Cost,_NewPos,OPos) % use bfs as goal pos is unknown
;otherwise -> G0 is 0, % else use Astar search
              map_distance(P,Goal,H0), % original estimate of cost to get to goal
              F0 is G0+H0,
              solve_task_Astar(Task,[[c(F0,G0,P),P]],0,R,Cost,_NewPos)

findOracle() :-
  solve_task_bfs(Task,[[c(0,0,P),P]],0,R,Cost,_NewPos,OPos), %return pos of nearest charging point
  G0 is 0, % else use Astar search
  map_distance(P,Goal,H0), % original estimate of cost to get to goal
  F0 is G0+H0,
  solve_task_Astar(Task,[[c(F0,G0,P),P]],0,R,Cost,_NewPos)..

findChargingPoint() :-
  solve_task_bfs(Task,[[c(0,0,P),P]],0,R,Cost,_NewPos,OPos),
  G0 is 0, % else use Astar search
  map_distance(P,Goal,H0), % original estimate of cost to get to goal
  F0 is G0+H0,
  solve_task_Astar(Task,[[c(F0,G0,P),P]],0,R,Cost,_NewPos).

append(oracle,VisitedOracles, NewVisitedOracles),

% Breadth-first search for o(Goal) and c(Goal) as don't know location of oracles and charging points
solve_task_bfs(Task,Agenda,Depth,RPath,[cost(Cost),depth(Depth)],NewPos,OPos) :-
  getClosestOraclePos(find(O),Current,RPath,Cost,NewPos,OPos) :-
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

%Call bfs to find all charging stations/ oracles
%Then call Astar to see if you can access it, if not call Astar on the next closest one
%Add visited oracles to list so you don't revisit them

%always keep track of of closest charging station
%keep looking for oracles until you are at a level of fuel = distanceToClosestChargeStation + 1 + 10 (price of oracle query)
