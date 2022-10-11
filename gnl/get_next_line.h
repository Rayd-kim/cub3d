/* ************************************************************************** */
/*                                                                            */
/*                                                        :::      ::::::::   */
/*   get_next_line.h                                    :+:      :+:    :+:   */
/*                                                    +:+ +:+         +:+     */
/*   By: youskim <youskim@student.42seoul.kr>       +#+  +:+       +#+        */
/*                                                +#+#+#+#+#+   +#+           */
/*   Created: 2022/01/07 22:57:29 by youskim           #+#    #+#             */
/*   Updated: 2022/04/09 13:20:56 by youskim          ###   ########.fr       */
/*                                                                            */
/* ************************************************************************** */

#ifndef GET_NEXT_LINE_H
# define GET_NEXT_LINE_H

# include <unistd.h>
# include <stdlib.h>
# define OPEN_MAX 256
# define BUFFER_SIZE 4

int		null_strlen(char *s1);
char	*str_cut(char *str);
char	*str_join(char *s1, char *s2);
char	*str_cpy(char *s1);
int		gnl_strchr(char *str, char c);
char	*get_next_line(int fd);
#endif
