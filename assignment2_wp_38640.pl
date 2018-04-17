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
