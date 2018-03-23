candidate_number(38640).

solve_task(Task,Cost):-
  my_agent(Agent),
  query_world( agent_current_position, [Agent,P] ),
  %solve_task_bt(Task,[c(0,P),P],0,R,Cost,_NewPos),!,  % prune choice point for efficiency
  solve_task_A_star(Task,[c(0,P,0),P],0,R,Cost,_NewPos),!,
  reverse(R,[_Init|Path]),
  query_world( agent_do_moves, [Agent,Path] ).

solve_task_Astar(Task,Agenda,Depth,RPath,[cost(Cost),depth(Depth)],NewPos) :-
  achieved(Task,Agenda,RPath,Cost,NewPos).
solve_task_Astar(Task,Agenda,D,RPath,Cost,NewPos) :-
  Agenda = [[c(F,P,G)|RPath]|Paths],
  setof([c(F1,P1,G1),RPath],
    (search(P,P1,R,C),\+ memberchk(R,RPath),G1 is G+C,map_distance(P1,Task,H),F1 is G1+H),
    Children),
  %sort(Children,SChildren),
  D1 is D+1,
  %append(Children,Paths,newAgenda),
  mergelists(Children,Agenda,NewAgenda), %Can also just append and sort
  solve_task_Astar(Task,NewAgenda,D1,RR,Cost,NewPos).

getChildren(F,P,G,RPath) :-
  search(P,P1,R,C),
  \+ memberchk(R,RPath),
  G1 is G+C,
  map_distance(P1,Task,H),
  F1 is G1+H.

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
  Current = [c(Cost,NewPos)|RPath],
  ( Exit=none -> true
  ; otherwise -> RPath = [Exit|_]
  ).
achieved(find(O),Current,RPath,Cost,NewPos) :-
  Current = [c(Cost,NewPos)|RPath],
  ( O=none    -> true
  ; otherwise -> RPath = [Last|_],map_adjacent(Last,_,O)
  ).

search(F,N,N,1) :-
  map_adjacent(F,N,empty).
