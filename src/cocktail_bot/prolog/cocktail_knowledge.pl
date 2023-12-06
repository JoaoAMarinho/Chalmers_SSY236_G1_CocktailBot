:- module(cocktail_knowledge,
    [
	cocktail_recipe/2,
    get_instances_for_cocktail/4
    ]).

:- consult('instance_utils').

% Facts representing available ingredients
% ingredient(?Ingredient)

% TODO: really necessary??
ingredient(tequila).
ingredient(rum).

ingredient(soda).
ingredient(milk).

ingredient(salt).
ingredient(sugar).

ingredient(ice).

ingredient(mint_leaves).
ingredient(lime).
ingredient(apple).
ingredient(banana).
ingredient(coconut).
ingredient(pineapple).
ingredient(strawberry).

% Predicate defining cocktail recipes and their ingredients
% cocktail_recipe(?Cocktail_name, ?Cocktail_ingredients)
%
% @param Cocktail_name		    name of the cocktail
%
% @param Cocktail_ingredients	list of ingredients

cocktail_recipe(margarita,   ['Tequila', 'Lime', 'Salt', 'Ice']).
cocktail_recipe(mojito,      ['Rum', 'Mint_leaves', 'Lime', 'Sugar', 'Soda', 'Ice']).
cocktail_recipe(pina_colada, ['Rum', 'Coconut', 'Pineapple', 'Ice']).

cocktail_recipe(strawberry_banana, ['Strawberry', 'Banana', 'Milk', 'Ice']).
cocktail_recipe(apple_strawberry,  ['Apple', 'Strawberry', 'Ice']).


% blabla
% get_instances_for_cocktail(+Cocktail, -Ingredients, -Ingredient_inst, -Alternative_inst)
% smth
%
% @param
get_instances_for_cocktail(Cocktail, Ingredients, Ingred_inst, Alt_inst) :-
    cocktail_recipe(Cocktail, Ingredients), !,
    write('Cocktail ['), write(Cocktail), write('] ingredients: '), write(Ingredients), nl,
    get_instances_for_ingredients(Ingredients, Ingred_inst, Alt_inst).

% In case the recipe does not exist
get_instances_for_cocktail(Cocktail, [], _, _) :-
    write('Cocktail ['), write(Cocktail),
    write('] is not present in the knowledge base!'), nl.

% Blabla
% smth
%
% @param
get_instances_for_ingredients([], [], []).
get_instances_for_ingredients([Ingred|Ingredients], [Instances_I|Ingred_inst], [Instances_A|Alt_inst]) :-
    get_instances_for_class(Ingred, Instances_I, Instances_A),
    get_instances_for_ingredients(Ingredients, Ingred_inst, Alt_inst).
