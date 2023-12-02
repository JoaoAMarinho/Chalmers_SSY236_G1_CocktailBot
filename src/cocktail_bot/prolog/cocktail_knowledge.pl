:- module(cocktail_knowledge,
    [
	cocktail_recipe/2
    ]).

% Facts representing available ingredients
% ingredient(?Ingredient)

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

cocktail_recipe(margarita,   [tequila, lime, salt, ice]).
cocktail_recipe(mojito,      [rum, mint_leaves, lime, sugar, soda, ice]).
cocktail_recipe(pina_colada, [rum, coconut, pineapple, ice]).

cocktail_recipe(strawberry_banana, [strawberry, banana, milk, ice]).
cocktail_recipe(apple_strawberry,  [apple, strawberry, ice]).