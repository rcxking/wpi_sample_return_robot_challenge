-- *******************************************************************
--  DatabaseSchema: Script for creating the database
--   Usage:
--       $ sqlite3 LTM.db < DatabaseSchema.sql
--
-- *******************************************************************

-- *******************************************************************
-- CLEAN
-- *******************************************************************
/*DROP TABLE Node;*/

-- *******************************************************************
-- CREATE
-- *******************************************************************
CREATE TABLE Node (
	id INTEGER NOT NULL,
	map_id INTEGER NOT NULL,
	weight INTEGER,
	pose BLOB,
	time_enter DATE,
	PRIMARY KEY (id)
);

CREATE TABLE Image (
	id INTEGER NOT NULL,
	data BLOB,
	time_enter DATE,
	PRIMARY KEY (id)
);

CREATE TABLE Depth (
	id INTEGER NOT NULL,
	data BLOB,   -- CV_32FC1, width = Image/raw_width, height=Image/raw_height
	constant FLOAT,
	local_transform BLOB,
	data2d BLOB,   -- CV_32FC2, Example: Laser scan
	time_enter DATE,
	PRIMARY KEY (id)
);

CREATE TABLE Link (
	from_id INTEGER NOT NULL,
	to_id INTEGER NOT NULL,
	type INTEGER NOT NULL, -- neighbor=0, loop=1, child=2
	transform BLOB,
	FOREIGN KEY (from_id) REFERENCES Node(id),
	FOREIGN KEY (to_id) REFERENCES Node(id)
);

-- 
CREATE TABLE Word (
	id INTEGER NOT NULL,
	descriptor_size INTEGER NOT NULL,
	descriptor BLOB NOT NULL,
	time_enter DATE,
	PRIMARY KEY (id)
);

CREATE TABLE Map_Node_Word (
	node_id INTEGER NOT NULL,
	word_id INTEGER NOT NULL,
	pos_x FLOAT NOT NULL,
	pos_y FLOAT NOT NULL,
	size INTEGER NOT NULL,
	dir FLOAT NOT NULL,
	response FLOAT NOT NULL,
	depth_x FLOAT,
	depth_y FLOAT,
	depth_z FLOAT,
	FOREIGN KEY (node_id) REFERENCES Node(id),
	FOREIGN KEY (word_id) REFERENCES Word(id)
);

CREATE TABLE Statistics (
	STM_size INTEGER,
	last_sign_added INTEGER,
	process_mem_used INTEGER,
	database_mem_used INTEGER,
	dictionary_size INTEGER,
	time_enter DATE
);

-- *******************************************************************
-- TRIGGERS
-- *******************************************************************
CREATE TRIGGER insert_Map_Node_Word BEFORE INSERT ON Map_Node_Word 
WHEN NOT EXISTS (SELECT Node.id FROM Node WHERE Node.id = NEW.node_id)
BEGIN
 SELECT RAISE(ABORT, 'Foreign key constraint failed in Map_Node_Word table');
END;

 --   Creating a trigger for time_enter
CREATE TRIGGER insert_Node_timeEnter AFTER INSERT ON Node
BEGIN
 UPDATE Node SET time_enter = DATETIME('NOW')  WHERE rowid = new.rowid;
END;

CREATE TRIGGER insert_Word_timeEnter AFTER INSERT ON Word
BEGIN
 UPDATE Word SET time_enter = DATETIME('NOW')  WHERE rowid = new.rowid;
END;

CREATE TRIGGER insert_Statistics_timeEnter AFTER INSERT ON Statistics
BEGIN
 UPDATE Statistics SET time_enter = DATETIME('NOW')  WHERE rowid = new.rowid;
END;


-- *******************************************************************
-- INDEXES
-- *******************************************************************
CREATE INDEX IDX_Map_Node_Word_node_id on Map_Node_Word (node_id);
CREATE INDEX IDX_Link_from_id on Link (from_id);

