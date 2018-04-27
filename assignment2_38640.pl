candidate_number(38640).

solve_task(Task,Cost):-
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

% A-star search for go(Goal)
solve_task_Astar(Task,Agenda,Depth,RPath,[cost(Cost),depth(Depth)],NewPos) :-
  achieved(Task,Agenda,RPath,Cost,NewPos).
solve_task_Astar(Task,Agenda,D,RR,Cost,NewPos) :-
  Agenda = [[c(F,G,P)|RPath]|Paths],
  Task = go(Goal),
  getChildrenAstar(F,G,P,RPath,Paths,Goal,SChildren),
  writeln('got children'),
  D1 is D+1,
  append(Paths,SChildren,NewAgenda), sort(NewAgenda, SNewAgenda), % append and sort is equivalent to merge
  solve_task_Astar(Task,SNewAgenda,D1,RR,Cost,NewPos).

% Breadth-first search for o(Goal) and c(Goal) as don't know location of oracles and charging points
solve_task_bfs(Task,Agenda,Depth,RPath,[cost(Cost),depth(Depth)],NewPos) :-
  achieved(Task,Agenda,RPath,Cost,NewPos).
solve_task_bfs(Task,Agenda,D,RR,Cost,NewPos) :-
  Agenda = [[c(F,F,P)|RPath]|Paths], % dont know position of o and c bfs -> no heuristic/ estimated cost to goal
  getChildrenBFS(F,P,RPath,Paths,Children),
  D1 is D+1,
  append(Paths, Children, NewAgenda),
  solve_task_bfs(Task,NewAgenda,D1,RR,Cost,NewPos).


getChildrenBFS(F,P,RPath,Paths,Children) :-
  findall([c(F1,F1,P1),R|RPath],
    (search(P,P1,R,C),
     \+memberchk(R,RPath),
     F1 is F+C,
     \+member([c(_,_,P1)|_],Paths)),
    Children).

getChildrenAstar(F,G,P,RPath,Paths,Goal,SChildren) :-
  findall([c(F1,G1,P1),R|RPath], %first arg of findall is template of return value (what the return value should look like)
    (search(P,P1,R,C), % find all children of curr pos
     \+ memberchk(R,RPath), % check that returned child isnt already in path
     G1 is G+C, % G is cost to arrive to curr pos, update G with cost to move to child
     map_distance(P1,Goal,H), % calc H that is heuristic (estimated cost) to get to goal
     F1 is G1+H,
     \+memberchk([c(_,_,P1)|_],Paths)), % update F which is total estimated cost to get to goal
    Children), % store result in Children
  sort(Children,SChildren).

mergelists([],[],[]).

mergelists([X],[],[X]).

mergelists([],[Y],[Y]).

mergelists([X|List1],[Y|List2],[X|List]) :-
  X =< Y,
  !,
  mergelists(List1,[Y|List2],List).

mergelists([X|List1],[Y|List2],[Y|List]) :-
  mergelists([X|List1],List2,List).

%%%%%%%%%% Useful predicates %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% backtracking depth-first search, needs to be changed to agenda-based A*
solve_task_bt(Task,Current,Depth,RPath,[cost(Cost),depth(Depth)],NewPos) :-
  achieved(Task,Current,RPath,Cost,NewPos).
solve_task_bt(Task,Current,D,RR,Cost,NewPos) :-
  Current = [c(F,P)|RPath],
  search(P,P1,R,C),
  \+ memberchk(R,RPath),  % check we have not been here already
  D1 is D+1,
  F1 is F+C,
  solve_task_bt(Task,[c(F1,P1),R|RPath],D1,RR,Cost,NewPos).  % backtrack search

achieved(go(Exit),Current,RPath,Cost,NewPos) :-
  Current = [[c(Cost,_,NewPos)|RPath]|_],
  ( Exit=none -> true
  ; otherwise -> RPath = [Exit|_]
  ).
achieved(find(O),Current,RPath,Cost,NewPos) :-
  Current = [[c(Cost,_,NewPos)|RPath]|_],
  ( O=none    -> true
  ; otherwise -> RPath = [Last|_],map_adjacent(Last,_,O)
  ).

search(F,N,N,1) :-
  map_adjacent(F,N,empty).
