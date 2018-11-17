/*
 * Postgresql: Setup Auvsi Imaging database
 * NOTE: this script assumes you're starting from
 *       a fresh install of postgres, and may cause
 *       cause errors other wise
 */

/* Step 1: create user `imaging_server` and database `auvsi` */
CREATE USER imaging_server WITH ENCRYPTED PASSWORD 'Byuauvs1';
CREATE DATABASE auvsi;-- WITH OWNER imaging_server;
GRANT CONNECT ON DATABASE auvsi TO imaging_server;
GRANT USAGE ON SCHEMA public TO imaging_server;
-- GRANT ALL PRIVILEGES ON ALL TABLES IN SCHEMA public TO imaging_server;
-- GRANT ALL PRIVILEGES ON ALL SEQUENCES IN SCHEMA public TO imaging_server;
-- connect to the new database user
\c auvsi;

/* Step 2: create all the tables */

CREATE TABLE "public"."incoming_image" (
  id serial NOT NULL,
  time_stamp timestamp NOT NULL,
  image_path text NOT NULL,
  manual_tap boolean NOT NULL default FALSE,
  autonomous_tap boolean NOT NULL default FALSE,
  PRIMARY KEY ("id")
);

CREATE TABLE "public"."incoming_gps" (
  id serial NOT NULL,
  time_stamp timestamp NOT NULL,
  latitude real NOT NULL,
  longitude real NOT NULL,
  altitude real NOT NULL,
  PRIMARY KEY ("id")
);

CREATE TABLE "public"."incoming_state" (
  id serial NOT NULL,
  time_stamp timestamp NOT NULL,
  roll real NOT NULL,
  pitch real NOT NULL,
  yaw real NOT NULL,
  PRIMARY KEY ("id")
);

CREATE TABLE "public"."manual_cropped" (
  id serial NOT NULL,
  raw_id int,
  time_stamp timestamp NOT NULL,
  cropped_path text NOT NULL,
  crop_coordinate_tl point,
  crop_coordinate_br point,
  tapped boolean NOT NULL default FALSE,
  PRIMARY KEY ("id")
);

CREATE TABLE "public"."outgoing_manual" (
  id serial NOT NULL,
  cropped_id int NOT NULL,
  type text NOT NULL CHECK(type = 'Standard' OR type = 'off_axis' OR type = 'emergent'),
  latitude real NOT NULL,
  longitude real NOT NULL,
  orientation text NOT NULL CHECK(orientation = 'N' OR orientation = 'NE' OR orientation = 'E' OR orientation = 'SE' OR orientation = 'S' OR orientation = 'SW' OR orientation = 'W' OR orientation = 'NW'),
  shape text NOT NULL CHECK(shape = 'circle' OR shape = 'semicircle' OR shape = 'quarter_circle' OR shape = 'triangle' OR shape = 'square' OR shape = 'rectangle' OR shape = 'trapezoid' OR shape = 'pentagon' OR shape = 'hexagon' OR shape = 'heptagon' OR shape = 'octagon' OR shape = 'star' OR shape = 'cross'),
  background_color text NOT NULL CHECK(background_color = 'white' OR background_color = 'black' OR background_color = 'gray' OR background_color = 'red' OR background_color = 'blue' OR background_color = 'green' OR background_color = 'yellow' OR background_color = 'purple' OR background_color = 'brown' OR background_color = 'orange'),
  alphanumeric text NOT NULL,
  alphanumeric_color text NOT NULL CHECK(alphanumeric_color = 'white' OR alphanumeric_color = 'black' OR alphanumeric_color = 'gray' OR alphanumeric_color = 'red' OR alphanumeric_color = 'blue' OR alphanumeric_color = 'green' OR alphanumeric_color = 'yellow' OR alphanumeric_color = 'purple' OR alphanumeric_color = 'brown' OR alphanumeric_color = 'orange'),
  description text default '',
  cropped_path text NOT NULL,
  submitted boolean NOT NULL default FALSE,
  PRIMARY KEY ("id")
);

CREATE TABLE "public"."outgoing_autonomous" (
  id serial NOT NULL,
  type text NOT NULL CHECK(type = 'Standard' OR type = 'off_axis' OR type = 'emergent'),
  latitude real NOT NULL,
  longitude real NOT NULL,
  orientation text NOT NULL CHECK(orientation = 'N' OR orientation = 'NE' OR orientation = 'E' OR orientation = 'SE' OR orientation = 'S' OR orientation = 'SW' OR orientation = 'W' OR orientation = 'NW'),
  shape text NOT NULL CHECK(shape = 'circle' OR shape = 'semicircle' OR shape = 'quarter_circle' OR shape = 'triangle' OR shape = 'square' OR shape = 'rectangle' OR shape = 'trapezoid' OR shape = 'pentagon' OR shape = 'hexagon' OR shape = 'heptagon' OR shape = 'octagon' OR shape = 'star' OR shape = 'cross'),
  background_color text NOT NULL CHECK(background_color = 'white' OR background_color = 'black' OR background_color = 'gray' OR background_color = 'red' OR background_color = 'blue' OR background_color = 'green' OR background_color = 'yellow' OR background_color = 'purple' OR background_color = 'brown' OR background_color = 'orange'),
  alphanumeric text NOT NULL,
  alphanumeric_color text NOT NULL CHECK(alphanumeric_color = 'white' OR alphanumeric_color = 'black' OR alphanumeric_color = 'gray' OR alphanumeric_color = 'red' OR alphanumeric_color = 'blue' OR alphanumeric_color = 'green' OR alphanumeric_color = 'yellow' OR alphanumeric_color = 'purple' OR alphanumeric_color = 'brown' OR alphanumeric_color = 'orange'),
  description text default '',
  cropped_path text NOT NULL,
  submitted boolean NOT NULL default FALSE,
  PRIMARY KEY ("id")
);

ALTER DATABASE auvsi OWNER TO imaging_server;
GRANT SELECT, INSERT, UPDATE, DELETE ON ALL TABLES IN SCHEMA public TO imaging_server;
GRANT USAGE, SELECT ON ALL SEQUENCES IN SCHEMA public TO imaging_server;
ALTER DEFAULT PRIVILEGES FOR USER imaging_server IN SCHEMA public GRANT SELECT, INSERT, UPDATE, DELETE ON TABLES TO imaging_server;

/* If you want to destroy the auvsi_imaging user here's how you do it: */
-- DROP OWNED BY imaging_server;
-- REVOKE ALL PRIVILEGES ON ALL TABLES IN SCHEMA public FROM imaging_server;
-- REVOKE ALL PRIVILEGES ON ALL SEQUENCES IN SCHEMA public FROM imaging_server;
-- REVOKE ALL PRIVILEGES ON ALL FUNCTIONS IN SCHEMA public FROM imaging_server;
-- DROP USER imaging_server;
