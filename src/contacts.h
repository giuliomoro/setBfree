#ifndef CONTACTS_H
#define CONTACTS_H

#define MAX_CONTACTS 549
#define NOF_CONTACTS_PER_KEY 9
#define DEFAULT_BUS 2
static inline int make_contact(short bus, short key){
  return (int)key * NOF_CONTACTS_PER_KEY + (int)bus;
}
static inline short get_contact_bus(int contact){
  return contact % NOF_CONTACTS_PER_KEY;
}
static inline int get_contact_key(int contact){
  return contact / NOF_CONTACTS_PER_KEY;
}

#endif /* CONTACTS_H */
