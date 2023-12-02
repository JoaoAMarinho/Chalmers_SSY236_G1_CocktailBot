# Ontology updates

## Classes 
### TemporalThing > Recipe 
Recipe
	Cocktail
		Margarita
		Mojito
		PinaColada
	Smoothie
		AppleStrawberry
		StrawberryBanana


### SpatialThing > EdibleStuff > FoodOrDrinkOrIngredient > FoodOrDrink > Drinks
Drink
	AlcoholicBeverage
		Rum
		Tequila
	Milk
	Soda

### SpatialThing > EdibleStuff > FoodOrDrinkOrIngredient > FoodOrDrink > Food > FruitOrVegetableFood
FruitOrVegetableFood
	EdibleFruit
		Apple
		Banana
		CitrusFruit
			Lime
		Coconut
		Pineapple
		Strawberry

### SpatialThing > EdibleStuff > FoodOrDrinkOrIngredient > FoodIngredientOnly
FoodIngredientOnly
	Cold
		Ice
	Condiment
		Salt
		Sugar


### SpatialThing > Container > ContainerArtifact > Holder 
Holder
	FoodShelf
	Table

### SpatialThing > Container > ContainerArtifact > FoodVessel > FoodStorageContainer
FoodStorageContainer
	Fridge

### SpatialThing > Container > ContainerArtifact > FoodVessel > DrinkingVessel
DrinkingVessel
	DrinkingBottle
	DrinkingGlass
	

## Relations

### Recipes
recipeProperties
    smoothieIngredient
    smoothieContainer
    cocktailIngredient
    cocktailContainer

### StorageFresh
freshStorage
domain: Table
range: Food

### StorageCold
coldStorage
domain: Fridge
range: Cold

### StorageCondiments
condimentsStorage
domain: FoodShelf
range: Condiment
