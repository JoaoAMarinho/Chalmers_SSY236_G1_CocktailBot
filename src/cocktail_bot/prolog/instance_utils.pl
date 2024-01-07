%%
%% Copyright (C) 2010 by Karinne Ramirez-Amaro
%%
%% This program is free software; you can redistribute it and/or modify
%% it under the terms of the GNU General Public License as published by
%% the Free Software Foundation; either version 3 of the License, or
%% (at your option) any later version.
%%
%% This program is distributed in the hope that it will be useful,
%% but WITHOUT ANY WARRANTY; without even the implied warranty of
%% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%% GNU General Public License for more details.
%%
%% You should have received a copy of the GNU General Public License
%% along with this program.  If not, see <http://www.gnu.org/licenses/>.
%%
/* ***************************************
	Author:	Karinne Ramirez-Amaro
	E-mail:	karinne@chalmers.se
	
 This library contains predicates used for the
 inference method using prolog queries.


 NOTE: The following symbols are used to describe the parameter
of the predicates
 + The argument is instantiated at entry.
 - The argument is not instantiated at entry.
 ? The argument is unbound or instantiated at entry.

*/

:- module(instance_utils,
    [
	create_instance_from_class/3,
	get_class_path/2,
	get_class/1,
	get_instances_for_class/3
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/rdf_db')).

:- rdf_db:rdf_register_ns(ssy236Ontology, 'http://www.chalmers.se/ontologies/ssy236Ontology.owl#', [keep(true)]).

%%%%%%%%%%%%%% Custom computables %%%%%%%%%%%%%%%%%%%%%%

% This function will create an instance of a desired class
% create_instance_from_class(+Class, +Instance)
% The created instance will have the predicate/property rdf:type
% to correctly inheritate the properties of its super-classes
%
% @param Class		represents the name of the class where the instance will be created.
%					Class could be of two forms:
%					Class='Orange'  <- make sure this class exist in the ontology "ssy236Ontology"
% @param Instance	asserted new instance

create_instance_from_class(Class, Instance) :-
	% Check ID and class path
	get_class_path(Class, Class_path),

	% assert/create the new instance
	rdf_assert(Instance, rdf:type, Class_path),
	write('New instance created: '), write(Instance), nl.

% This function will return the path of the class/instance given
% get_class_path(+Class, ?Class_path)
%
% @param Class		represents the name of the class where the instance will be created.
%					Class could be of two forms:
%					Class='Orange'  <- make sure this class exist in the ontology "ssy236Ontology"
% @param Class_path	correct class full path

get_class_path(Class, Class_path):-
	((concat_atom(List, '#', Class),length(List,Length),Length>1) ->
	( % Class has already a URI
	   Class_path=Class );
	  % When the class does not have URL, will get the knowrob path
        ( TempClass='http://www.chalmers.se/ontologies/ssy236Ontology.owl#',
	atom_concat(TempClass, Class, Class_path)
	% write(Class_path), nl
 	)).

% This function will return the path of the obtained class
% get_class(+Class)
%
% @param Class		represents the name of the class that we ask to create (if it does not exsist yet).
%					Class could be of two forms:
%					Class='Orange'  <- make sure this class exist in the ontology "ssy236Ontology"

get_class(Class):-
	get_class_path(Class,Class_path),
	rdf_has(Class_path, rdf:type, owl:'Class'), !,
	write(Class), write(' already exists!'), nl, fail.

get_class(Class):-
	get_class_path(Class,Class_path),
	rdf_assert(Class_path, rdf:type, owl:'Class'),
	write('New class created: '), write(Class), nl.

% This function will return the instances of a given class
% get_instances_for_class(+Class, -Class_inst, -Alt_inst)
%
% @param Class			represents the name of the class to get the instances
% @param Class_inst		list of instances for the given class
% @param Alt_inst		list of alternative (storage) instances for the given class

get_instances_for_class(Class, Class_inst, Alt_inst) :-
	get_class_path(Class, Class_path),
	rdf_has(Class_path, rdf:type, owl:'Class'),
	(	setof(X, rdfs_individual_of(X, Class_path), Individuals) -> Class_inst = Individuals
	;	Class_inst = []),
	write(Class), write(' instances: '), write(Class_inst), nl,
	get_storage_for_class(Class_path, Storage),
	(	setof(X, rdfs_individual_of(X, Storage), Storages) -> Alt_inst = Storages
	;	Alt_inst = []),
	write('Storage instances: '), write(Alt_inst), nl, nl.


% This function will return the storage class for a given item class
% get_storage_for_class(+Class, -Storage)
%
% @param Class			represents the name of the class to get the storage
% @param Storage		storage class

get_storage_for_class(Class, Storage) :-
	( rdf_has(ssy236Ontology:freshStorage, rdfs:range, Class);
	  rdf_has(ssy236Ontology:coldStorage, rdfs:range, Class);
	  rdf_has(ssy236Ontology:condimentsStorage, rdfs:range, Class);
	  rdf_has(ssy236Ontology:glassStorage, rdfs:range, Class) ),
	rdf_has(ssy236Ontology:freshStorage, rdfs:domain, Storage), !.

% If the storage class is not found, check the super class
get_storage_for_class(Class, Storage) :-
	rdf_has(Class, rdfs:subClassOf, SuperClass),
	get_storage_for_class(SuperClass, Storage).